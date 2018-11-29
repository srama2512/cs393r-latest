"""Simple behavior that stands, kicks, and then sits down."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import sys
import math
import core
import pose
import memory
import UTdebug
import commands
import cfgstiff
import mem_objects

from pid_controller import *
from memory import joint_commands
from state_machine import Node, S, T, F, C, LoopingStateMachine, StateMachine
from pose import BlockLeftAndStand, BlockRightAndStand, Squat, BlockCenterStand, Sit#BlockLeftStand, BlockRightStand


list_of_beacons = [core.WO_BEACON_BLUE_YELLOW,
                   core.WO_BEACON_YELLOW_BLUE,
                   core.WO_BEACON_BLUE_PINK,
                   core.WO_BEACON_PINK_BLUE,
                   core.WO_BEACON_PINK_YELLOW,
                   core.WO_BEACON_YELLOW_PINK]

def getBeaconCounts():
    num_seen_beacons = 0
    for b in list_of_beacons:
        beacon = mem_objects.world_objects[b]
        if beacon.seen:
            num_seen_beacons += 1
    return num_seen_beacons

def sgn(x):
    return -1.0 if x < 0 else 1.0

def sgn_val(x, thresh):
    assert thresh > 0

    if x > thresh:
        return 1.0
    elif x < -thresh:
        return -1.0
    else:
        return 0.0

class EWMA():
    # Exponentially weighted moving average
    def __init__(self, val, gamma):
        self.init_val = val
        self.val = val
        self.gamma = gamma

    def update(self, val):
        self.val = self.gamma * self.val + (1-self.gamma) * val
        return self.val

    def get(self):
        return self.val

    def reset(self):
        self.val = self.init_val

class LocEWMA():
    def __init__(self, val, gamma):
        self.loc = list((EWMA(v, gamma) for v in val)) # tuple
    
    def update(self, val):
        output = []
        for i, l in enumerate(self.loc):
            output.append(l.update(val[i]))

        return tuple(output)


def updateBeaconDict(beacon_dict):
    num_seen_beacons = 0
    for b in list_of_beacons:
        beacon = mem_objects.world_objects[b]
        if beacon.seen:
            beacon_dict[b] += 1

class Playing(LoopingStateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 3.0:
                self.finish()

    class Kick(Node):
        def run(self):
            if self.getFrames() <= 3:
                memory.walk_request.noWalk()
                memory.kick_request.setFwdKick()
            if self.getFrames() > 10 and not memory.kick_request.kick_running_:
                self.finish()

    class Walk(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0)

    class WalkToTarget(Node):
        def __init__(self, d_threshold=250.0, d_target_compensation=200.0):
            super(Playing.WalkToTarget, self).__init__()
            self.d_threshold = d_threshold
            self.d_target_compensation = d_target_compensation
            self.last_seen = 0
            self.pid_position = PIDPosition()
            self.pid_position.max_t_res = 0.2
            self.pid_position.set_const_x(1.2e-3, 0., 0.)
            self.pid_position.set_const_y(4.5e-3, 1e-4, 0., 1000.0)
            self.pid_position.set_const_t(1.2, 0., 0.)

        def run(self):
            print('===> WalkToTarget: entered run!')
            ball = memory.world_objects.getObjPtr(core.WO_BALL)

            if ball.seen:
                core.ledsC.frontRightEar(1)
                core.ledsC.backRightEar(1)
                core.ledsC.frontLeftEar(0)
                core.ledsC.backLeftEar(0)

                if ball.visionDistance < self.d_threshold:
                    print('===> WalkToTarget: reached the ball!')
                    commands.setWalkVelocity(0., 0., 0.)
                    self.finish()
                else:
                    print('===> WalkToTarget: visionDistance: {}   visionBearing: {}'.format(ball.visionDistance, ball.visionBearing))
                    vel_x, vel_y, vel_t = self.pid_position.update((0, 0, 0), (ball.visionDistance + self.d_target_compensation, 0.0, ball.visionBearing))
                    commands.setWalkVelocity(vel_x, vel_y, vel_t)       

                self.last_seen = self.getTime()
            else:
                if self.getTime() - self.last_seen > 2.0:
                    commands.setWalkVelocity(0., 0., 0.)
                    self.postFailure()

    class HeadPos(Node):
        """Changes the head pos to the desired pan and tilt"""
        def __init__(self, pan=0, tilt=0, duration=2.0):
            """
            pan: (left/right) in degrees
            tilt: (up/down) in degrees
            duration: time in seconds
            """
            super(Playing.HeadPos, self).__init__()
            self.pan = pan * core.DEG_T_RAD
            self.tilt = tilt
            self.duration = duration 
        def run(self):
            commands.setHeadPanTilt(pan=self.pan, tilt=self.tilt, time=self.duration)

    class PositionToKick(Node):
        def __init__(self, goal_b_threshold=0.1, goal_b_offset=-0.3, ball_x_threshold=200.0, ball_y_offset=-75.0, ball_y_threshold=7.0):
            super(Playing.PositionToKick, self).__init__()
            self.goal_b_threshold = goal_b_threshold
            self.goal_b_offset = goal_b_offset
            self.ball_x_threshold = ball_x_threshold
            self.ball_y_offset = ball_y_offset
            self.ball_y_threshold = ball_y_threshold
            self.last_seen = 0

            self.forward_compensation = 100.0
            self.pid_position = PIDPosition()
            self.pid_position.set_const_x(1.5e-3, 1e-5, 0., 1000.0)
            self.pid_position.set_const_y(3.0e-3, 1e-4, 0., 1000.0)
            self.pid_position.set_const_t(1.2, 0., 0.)
            self.pid_position.max_t_res = 0.2

        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)

            if goal.seen and ball.seen:

                ball_side_dist = ball.visionDistance * math.sin(ball.visionBearing)
                ball_fwd_dist = ball.visionDistance * math.cos(ball.visionBearing)
                print('===> PositionToKick: visionDistanceGoal: {}   visionBearingGoal: {}   ball_side_dist: {}   ball_fwd_dist: {}'.format(goal.visionDistance, 
                                                                                                                                               goal.visionBearing, 
                                                                                                                                               ball_side_dist,
                                                                                                                                               ball_fwd_dist))

                if abs(goal.visionBearing - self.goal_b_offset) < self.goal_b_threshold and abs(ball_side_dist - self.ball_y_offset) < self.ball_y_threshold and abs(ball_fwd_dist) < self.ball_x_threshold:
                    print('===> PositionToKick: Reached within bearing of the goal and distance of the ball!')
                    print('===> PositionToKick: FINAL - visionDistanceGoal: {}   visionBearingGoal: {}   ball_side_dist: {}   ball_fwd_dist: {}'.format(goal.visionDistance, 
                                                                                                                                               goal.visionBearing, 
                                                                                                                                               ball_side_dist,
                                                                                                                                               ball_fwd_dist))
                    commands.setWalkVelocity(0.0, 0.0, 0.0)
                    self.finish()
                else:
                    (vel_x, vel_y, vel_t) = self.pid_position.update((0, 0, 0), (ball_fwd_dist, ball_side_dist - self.ball_y_offset, goal.visionBearing - self.goal_b_offset))
                    print ("===> PositionToKick: vel_x : ", vel_x, "vel_y :", vel_y, "vel_t : ", vel_t)
                    commands.setWalkVelocity(vel_x, vel_y, vel_t)
            else:
                print('===> PositionToKick: Not seeing either the ball (or) the goal')
                if self.getTime() - self.last_seen > 2.0:
                    self.postFailure()

    class Dribble(Node):
        def __init__(self, goal_b_threshold=0.15, goal_x_threshold=2000.0, ball_x_threshold=220.0, ball_y_threshold=50.0, vel_x=0.5, vel_y=0.6, omega=0.1):
            self.goal_b_threshold = goal_b_threshold
            self.goal_x_threshold = goal_x_threshold
            self.ball_x_threshold = ball_x_threshold
            self.ball_y_threshold = ball_y_threshold
            self.last_seen = 0

            self.pid_position = PIDPosition()
            self.pid_position.max_t_res = 0.2
            self.pid_position.set_const_x(1.2e-3, 0., 0.)
            self.pid_position.set_const_y(4.5e-3, 1e-4, 0., 1000.0)
            self.pid_position.set_const_t(1.2, 0., 0.)
            self.ball_fwd_compensation = 150
            self.ball_fwd_gap = 200

            self.goal_b_ewma = EWMA(0., 0.95)
            self.goal_d_ewma = EWMA(10000., 0.80)

            self.beacon_dict = {k:0 for k in list_of_beacons}
            super(Playing.Dribble, self).__init__()

        def reset(self):
            super(Playing.Dribble, self).reset()
            self.beacon_dict = {k:0 for k in list_of_beacons}
            self.goal_b_ewma.reset()

        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)

            updateBeaconDict(self.beacon_dict)

            # if goal.seen and ball.seen:
            if ball.seen:
                self.last_seen = self.getTime()
                ball_side_dist = ball.visionDistance * math.sin(ball.visionBearing)
                ball_fwd_dist = ball.visionDistance * math.cos(ball.visionBearing)
                #sys.stdout.flush()
                if goal.seen:
                    goal_vision_bearing = goal.visionBearing
                    goal_fwd_dist = goal.visionDistance * math.cos(goal.visionBearing)
                else:
                    BY_cnt = self.beacon_dict[core.WO_BEACON_BLUE_YELLOW] + self.beacon_dict[core.WO_BEACON_YELLOW_BLUE]
                    BP_cnt = self.beacon_dict[core.WO_BEACON_PINK_BLUE] + self.beacon_dict[core.WO_BEACON_BLUE_PINK]
                    if BY_cnt > BP_cnt:
                        goal_vision_bearing = 1.0
                    else:
                        goal_vision_bearing = -1.0
                    goal_fwd_dist = self.goal_x_threshold + 1.0

                self.goal_b_ewma.update(goal_vision_bearing)
                self.goal_d_ewma.update(goal_fwd_dist)

                print('===> Dribble: visionDistanceGoal: {}   visionBearingGoal: {}   ball_side_dist: {}   ball_fwd_dist: {}'.format(self.goal_d_ewma.get(), 
                                                                                                                                               self.goal_b_ewma.get(), 
                                                                                                                                               ball_side_dist,
                                                                                                                                               ball_fwd_dist))
                if abs(self.goal_b_ewma.get()) < self.goal_b_threshold and abs(ball_side_dist) < self.ball_y_threshold:
                    print('===> Dribble: Reached within bearing of the goal and distance of the ball!')

                    if abs(self.goal_d_ewma.get()) < self.goal_x_threshold:
                        print('===> Dribble: Reached the goal!')
                        (vel_x, vel_y, vel_t) = (0., 0., 0.)
                        self.finish()
                    else:
                        (vel_x, vel_y, vel_t) = self.pid_position.update((0, 0, 0), (ball_fwd_dist + self.ball_fwd_compensation, \
                            ball_side_dist, self.goal_b_ewma.get()))
                else:
                    (vel_x, vel_y, vel_t) = self.pid_position.update((0, 0, 0), (ball_fwd_dist - self.ball_fwd_gap, ball_side_dist, self.goal_b_ewma.get()))

                commands.setWalkVelocity(vel_x, vel_y, vel_t)
            else:
                print('===> Dribble: Not seeing the ball')
                if self.getTime() - self.last_seen > 2.0:
                    commands.setWalkVelocity(0., 0., 0.)
                    self.postFailure()

    class RotateBody(Node):
        def __init__(self):
            super(Playing.RotateBody, self).__init__()

        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            if ball.seen:
                commands.setWalkVelocity(0., 0., 0.)
                self.finish()
            else:
                commands.setHeadPanTilt(pan=0., tilt=0., time=2.0)
                commands.setWalkVelocity(0., 0., 0.3)

    class SearchForBall(Node):
        def __init__(self, pan=0, tilt=0, duration=2.0):
            """
            pan: (left/right) in degrees
            tilt: (up/down) in degrees
            duration: time in seconds
            """
            super(Playing.SearchForBall, self).__init__()
            self.pan = pan * core.DEG_T_RAD
            self.tilt = tilt
            self.duration = duration 
        def run(self):
            """If the ball was seen, then move head towards the ball"""
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            if ball.seen:
                print ('Found ball')
                sys.stdout.flush()
                self.finish()
            if self.getTime() > self.duration:
                print ('Finished search')
                sys.stdout.flush()
                self.postFailure()

            commands.setHeadPanTilt(pan=self.pan, tilt=self.tilt, time=self.duration)

    class LookAtBall(Node):
        def __init__(self, delta_pan=6, omega=0.1, duration=3.0, ball_b_thresh=0.05):
            """
            duration: time in seconds
            """
            super(Playing.LookAtBall, self).__init__()
            self.duration = duration
            self.thresh = 20 # if the ball is 5 pixels away, only then move
            self.delta_pan = delta_pan * core.DEG_T_RAD
            self.omega = omega
            self.midX = 160
            self.midY = 120
            self.ball_seen = False
            self.ball_b_thresh = ball_b_thresh
            self.last_seen = 0

        def sgn(self, x):
            return 1.0 if x > 0.0 else -1.0

        def clip(self, x, xmin, xmax):
            return max(min(x, xmax), xmin)

        def run(self):
            """If the ball was seen, then move head towards the ball"""
            ball = memory.world_objects.getObjPtr(core.WO_BALL)

            if ball.seen:
                ballX = ball.imageCenterX
                ballY = ball.imageCenterY
                
                pan = core.joint_values[core.HeadYaw]
                if abs(ballX - self.midX) > self.thresh:
                   if ballX > self.midX:
                       pan -= self.delta_pan
                   else:
                       pan += self.delta_pan
                
                pan = self.clip(pan, -75.0*core.DEG_T_RAD, 75.0*core.DEG_T_RAD)      
                if abs(ball.visionBearing) > self.ball_b_thresh:
                    omega = self.sgn(ball.visionBearing) * self.omega
                else:
                    omega = 0.0

                commands.setHeadPan(pan, target_time=self.duration)
                commands.setWalkVelocity(0.0, 0.0, omega)

                if abs(ballX - self.midX) <= self.thresh and abs(ball.visionBearing) <= self.ball_b_thresh:
                    print('===> LookAtBall: looking at ball! Finishing')
                    self.finish()
                self.last_seen = self.getTime()
            else:
                if self.getTime() - self.last_seen > 2.0:
                    commands.setWalkVelocity(0.0, 0.0, 0.0)
                    self.postFailure()

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    class Unstiffen(Node):
        def run(self):
            memory.walk_request.noWalk()
            commands.setStiffness(cfgstiff.RightLegOffRestWalk)

    def setup(self):
        walk_to_target = self.WalkToTarget()
        stand = self.Stand()
        stand_at_ball = self.Stand()
        dribble = self.Dribble()
        ptk = self.PositionToKick()
        center = self.HeadPos(-22, 0)
        kick = self.Kick()
        lookatball = self.LookAtBall(delta_pan=12, duration=0.2)
        leftSearch = self.SearchForBall(75, -22, 2.0)
        rightSearch = self.SearchForBall(-75, -22, 2.0)
        rotateBody = self.RotateBody()

        # self.trans(stand, C, center, T(4.0), leftSearch, C, rightSearch, C, lookatball, C, walk_to_target, C, ptd, C, self.Stand(), C)
        
        self.add_transition(stand, C, center)
        self.add_transition(center, T(2.0), leftSearch)
        self.add_transition(leftSearch, C, rightSearch)
        self.add_transition(leftSearch, F, rightSearch)
        self.add_transition(rightSearch, C, lookatball)
        self.add_transition(rightSearch, F, rotateBody)
        self.add_transition(rotateBody, C, lookatball)
        self.add_transition(lookatball, C, walk_to_target)
        self.add_transition(lookatball, F, center)
        self.add_transition(walk_to_target, C, dribble)
        self.add_transition(walk_to_target, F, center)
        self.add_transition(dribble, C, ptk)
        self.add_transition(dribble, F, center)
        self.add_transition(ptk, C, stand_at_ball)
        self.add_transition(ptk, F, dribble)
        self.add_transition(stand_at_ball, C, kick)
        self.add_transition(kick, C, dribble)

        #self.trans(self.PositionToKick(), C)
        # self.trans(self.Stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
        # self.trans(self.Stand(), C, self.Kick(), C, self.Unstiffen(), C)

        # self.trans(self.Stand(), C, self.Unstiffen(), C)


class Testing(LoopingStateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()

    class HeadPos(Node):
        """Changes the head pos to the desired pan and tilt"""
        def __init__(self, pan=0, tilt=0, duration=2.0):
            """
            pan: (left/right) in degrees
            tilt: (up/down) in degrees
            duration: time in seconds
            """
            super(Testing.HeadPos, self).__init__()
            self.pan = pan * core.DEG_T_RAD
            self.tilt = tilt
            self.duration = duration 
        def run(self):
            commands.setHeadPanTilt(pan=self.pan, tilt=self.tilt, time=self.duration)

    class BlockKeeper(Node):
        def __init__(self):
            self.prev_x = 0
            self.prev_y = 0
            self.set_previous = False
            self.gamma_vel_x = 0.8
            self.gamma_vel_y = 0.5
            self.vel_x = EWMA(0.0, gamma=self.gamma_vel_x)
            self.vel_y = EWMA(0.0, gamma=self.gamma_vel_y)
            
            self.delta_time = 1.20
            self.y_dist_thresh = 100
            self.y_max_dist_thresh = 700
            self.miss_thresh = 2.0
            self.upper_thresh_x = 150.0
            self.upper_thresh_y = 80.0
            self.upper_thresh_t = 0.25 
            super(Testing.BlockKeeper, self).__init__()
            self.last_ball_seen = self.getTime()
            self.prev_time = self.getTime()

        def reset(self):
            super(Testing.BlockKeeper, self).reset()

            self.prev_x = 0.0
            self.prev_y = 0.0
            self.set_previous = False
            self.prev_time = self.getTime()
            self.vel_x.reset()
            self.vel_y.reset()
            self.last_ball_seen = self.getTime()

        def run(self):
            ball = mem_objects.world_objects[core.WO_BALL_KF]
            # commands.setHeadPan(ball.bearing, 0.1)
            commands.setHeadPanTilt(pan=0.0, tilt=-20.0, time=2.0)
            num_seen_beacons = getBeaconCounts()
            robot_state = mem_objects.memory.world_objects.getObjPtr(mem_objects.memory.robot_state.WO_SELF)
            locX, locY = robot_state.loc.x, robot_state.loc.y
            locTheta = robot_state.orientation
            print('===> Localization: locX: {:.3f},  locY: {:.3f},  locTheta: {:.3f}'.format(locX, locY, math.degrees(locTheta)))
            
            if ball.seen:
                ball_x = ball.distance * math.cos(ball.bearing)
                ball_y = ball.distance * math.sin(ball.bearing)

                
                duration = self.getTime() - self.prev_time + 1e-5
                print('===> Keeper: Ball seen: duration: {:.3f}, prev_x: {:.3f}, prev_y: {:.3f}, ball_x: {:.3f}, ball_y: {:.3f}  vel_x: {:.3f}  vel_y: {:.3f}'.format(\
                                                        duration, self.prev_x, self.prev_y, ball_x, ball_y, self.vel_x.get(), self.vel_y.get()))

                vel_x = (ball_x - self.prev_x) / duration
                vel_y = (ball_y - self.prev_y) / duration

                if self.set_previous:
                    self.vel_x.update(vel_x)
                    self.vel_y.update(vel_y)

                self.prev_time = self.getTime()
                self.prev_x = ball_x
                self.prev_y = ball_y


                if self.set_previous and (ball_x + self.vel_x.get() * self.delta_time < 0.0):
                    time_to_reach = (0 - ball_x) / (self.vel_x.get() + 1e-5)
                    print("===> Ball is close, blocking!  vel_x: {:.3f}  vel_y: {:.3f}".format(self.vel_x.get(), self.vel_y.get()))
                    y_pred = time_to_reach * self.vel_y.get() + ball_y
                    print('Predicted y at end of {} secs: {}'.format(time_to_reach, y_pred))

                    if abs(y_pred) < self.y_max_dist_thresh:

                        if y_pred > self.y_dist_thresh:
                            choice = "left"
                            print ('\n\n\n\n<----------------- LEFT\n\n\n\n')
                        elif y_pred < -self.y_dist_thresh:
                            choice = "right"
                            print ('\n\n\n\n                                      RIGHT ----------------->\n\n\n\n')
                        else:
                            choice = "center"
                            print ('\n\n\n\n<------------------------- CENTER --------------------------->\n\n\n\n')
                        self.postSignal(choice)
                elif self.set_previous:
                    # if abs(ball_x) > self.upper_thresh_x or abs(ball_y) > self.upper_thresh_y or abs(ball.bearing) > self.upper_thresh_t:
                    if abs(ball.bearing) > self.upper_thresh_t:
                        self.postSignal('strafe')

                
                self.set_previous = True

            else:
                if self.miss_thresh < self.getTime() - self.last_ball_seen:
                    self.postSignal('ball')

    class LookForBall(Node):
        def __init__(self, pan=0, tilt=0, duration=2.0):
            """
            pan: (left/right) in degrees
            tilt: (up/down) in degrees
            duration: time in seconds
            """
            super(Testing.LookForBall, self).__init__()
            self.headRotDir = 1.0
            self.headRotDur = 3.0
            self.headPanLimit = 60.0 * core.DEG_T_RAD
            self.lastShift = self.getTime()


        def run(self):
            """If the ball was seen, then move head towards the ball"""
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            if self.getTime() - self.lastShift > self.headRotDur:
                self.lastShift = self.getTime()
                self.headRotDir *= -1.0
            if ball.seen:
                self.finish()

            commands.setHeadPanTilt(pan=self.headPanLimit * self.headRotDir, tilt=-10.0, time=self.headRotDur)

    class DelayTimer(Node):
        def __init__(self, timer=0.5):
            super(Testing.DelayTimer, self).__init__()
            self.delay = timer
            self.start_time = self.getTime()

        def reset(self):
            super(Testing.DelayTimer, self).reset()
            self.start_time = self.getTime()

        def run(self):
            if self.getTime() - self.start_time > self.delay:
                self.finish()

    class StrafeBall(Node):
        def __init__(self):
            super(Testing.StrafeBall, self).__init__()

            self.action_dist_thresh = 1500
            self.x_dist_thresh = 20
            self.y_dist_thresh = 20
            self.d_line_fwd_comp = 150

            self.theta_thresh = 0.1
            self.last_seen = self.getTime()

            self.pid_position = PIDPosition()
            self.pid_position.set_const_x(1.2e-3, 0., 0.)
            self.pid_position.set_const_y(1.0e-3, 1e-4, 0., 1000.0)
            self.pid_position.set_const_t(2.4, 0., 0.)
            self.pid_position.max_t_res = 0.2

            self.pid_position.max_x_res = 0.3
            self.pid_position.max_y_res = 0.3

            self.lastShift = self.getTime()
            self.headRotDir = 1.0
            self.headRotDur = 1.0
            self.headPanLimit = 30.0 * core.DEG_T_RAD
            self.gcx = 1000.
            self.gcy = 0.
            self.loc_gamma = 0.95
            self.loc_ewma = LocEWMA((1000.0, 0.0, math.pi), self.loc_gamma)
            self.arc_radius = 500.0

            self.thresh_x = 100.0
            self.thresh_y = 50.0
            self.thresh_t = 0.15 


        def reset(self):
            super(Testing.StrafeBall, self).reset()
            self.lastShift = self.getTime()
            self.last_seen = self.getTime()
            # TODO - fix this
            # self.loc_ewma = LocEWMA((1000.0, 0.0, math.pi), self.loc_gamma)


        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            gline = memory.world_objects.getObjPtr(core.WO_GOAL_D_LINE)
            robot_state = mem_objects.memory.world_objects.getObjPtr(mem_objects.memory.robot_state.WO_SELF)
            locX, locY = robot_state.loc.x, robot_state.loc.y
            locTheta = robot_state.orientation

            timePassed = self.getTime() - self.lastShift
            if timePassed < 0.5 * self.headRotDur:
                self.headRotDir = 1.0
            elif timePassed < 1.5 * self.headRotDur:
                self.headRotDir = -1.0
            elif timePassed < 2.0 * self.headRotDur:
                self.headRotDir = 0.0
            elif timePassed > 6.0 * self.headRotDur:
                self.lastShift = self.getTime()

            # if self.getTime() - self.lastShift > self.headRotDur:
            #     # self.lastShift = self.getTime()
            #     self.prevDir = self.headRotDir
            #     self.headRotDir = 0.0

            # if self.getTime() - self.lastShift > self.headRotDur + 10:
            #     self.lastShift = self.getTime()
            #     self.headRotDir = -self.prevDir

            commands.setHeadPanTilt(pan=self.headPanLimit * self.headRotDir, tilt=-10.0, time=self.headRotDur)

            print ('===> StrafeBall: locX: {}, locY: {}, locTheta: {}'.format(locX, locY, locTheta))
            
            locXS, locYS, locThetaS = self.loc_ewma.update((locX, locY, locTheta))

            if ball.seen:
                self.last_seen = self.getTime()
                ball_x = ball.visionDistance * math.cos(ball.visionBearing)
                ball_y = ball.visionDistance * math.sin(ball.visionBearing)
                ball_t = ball.visionBearing
                locAlpha = math.atan2(ball.loc.y-self.gcy, ball.loc.x-self.gcx)
                target_x = self.gcx + self.arc_radius * math.cos(locAlpha)
                target_y = self.gcy + self.arc_radius * math.sin(locAlpha)
                target_theta = locAlpha


                print('===> StrafeBall: lxs: {:.3f}, lys: {:.3f}, lts: {:.3f}, tx: {:.3f}, ty: {:.3f}, tt: {:.3f}'.format(
                                        locXS, locYS, locThetaS, target_x, target_y, target_theta))
                (vel_x, vel_y, vel_t) = self.pid_position.update((0., 0., 0.), (locXS-target_x, locYS-target_y, ball.visionBearing))
                print('===> StrafeBall: vel_x: {:.3f}, vel_y: {:.3f}, vel_t: {:.3f}'.format(vel_x, vel_y, vel_t))
                # if ball.visionDistance < self.action_dist_thresh and (abs(ball_y) > self.y_dist_thresh \
                #     or abs(ball_t) > self.theta_thresh or abs(target_x) > self.x_dist_thresh):
                #     (vel_x, vel_y, vel_t) = self.pid_position.update((0, 0, 0), (target_x, ball_y, ball_t))
                # else:
                #     vel_x = 0.
                #     vel_y = 0.
                #     vel_t = 0.

                if gline.seen:
                    gline_x = gline.visionDistance * math.cos(gline.visionBearing) - self.d_line_fwd_comp
                    if gline_x < self.d_line_fwd_comp:
                        gl_target_x = gline_x - self.d_line_fwd_comp
                    else:
                        gl_target_x = ball_x
                    (vel_x, vel_y, vel_t) = self.pid_position.update((0., 0., 0.), (gl_target_x, 0., ball.visionBearing))
                # else:
                #     target_x = ball_x

                if self.pid_position.has_converged(self.thresh_x, self.thresh_y, self.thresh_t):
                    commands.setWalkVelocity(0.0, 0.0, 0.0)
                    commands.setHeadPanTilt(pan=0.0, tilt=-10.0, time=1.0)
                    self.finish()
                else:
                    commands.setWalkVelocity(vel_x, vel_y, vel_t)
                    print ('===> StrafeBall: GLine vel_x: {:.3f} Ball seen vel_y: {:.3f} vel_t: {:.3f}'.format(vel_x, vel_y, vel_t))

            else:
                print('===> StrafeBall: Not seeing the ball')
                if self.getTime() - self.last_seen > 2.0:
                    commands.setWalkVelocity(0., 0., 0.)
                    commands.setHeadPanTilt(pan=0.0, tilt=-10.0, time=1.0)
                    self.postSignal('ball')
                    #self.postFailure()

    def setup(self):
        blocker = self.BlockKeeper()
        blocks = {"left": BlockLeftAndStand(time=6.0),#BlockLeftStand(),
                  "right": BlockRightAndStand(time=6.0),#BlockRightStand(),
                  "center": pose.Sit()
                  }
        stand = self.Stand()
        strafe = self.StrafeBall()
        center = self.HeadPos(0, 0)
        look_for_ball = self.LookForBall()
        delay_timer = self.DelayTimer(0.5)

        self.add_transition(stand, C, strafe, S('ball'), look_for_ball, C, strafe)
        self.add_transition(blocker, S('ball'), look_for_ball)
        self.add_transition(blocker, S('strafe'), strafe)
        for name in blocks:
            b = blocks[name]
            self.add_transition(strafe, C, delay_timer, C, blocker, S(name), b, T(10), stand)

        # self.trans(Stand(), C, StrafeBall(), C, Stand(), C)

    