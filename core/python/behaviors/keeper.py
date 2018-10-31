"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import pdb
import core
import commands
import mem_objects
from state_machine import Node, S, T, F, C, LoopingStateMachine, StateMachine
import UTdebug
import math
import memory
from memory import joint_commands
from pose import BlockLeftAndStand, BlockRightAndStand, Squat, BlockCenterStand, Sit#BlockLeftStand, BlockRightStand
from pid_controller import *

def getBeaconCounts():
    list_of_beacons = [core.WO_BEACON_BLUE_YELLOW,
                           core.WO_BEACON_YELLOW_BLUE,
                           core.WO_BEACON_BLUE_PINK,
                           core.WO_BEACON_PINK_BLUE,
                           core.WO_BEACON_PINK_YELLOW,
                           core.WO_BEACON_YELLOW_PINK]
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
            super(HeadPos, self).__init__()
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
        super(BlockKeeper, self).__init__()
        self.last_ball_seen = self.getTime()
        self.prev_time = self.getTime()

    def reset(self):
        super(BlockKeeper, self).reset()

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

class ResetNode(Node):
    def __init__(self, t_node):
        super(ResetNode, self).__init__()
        self.t_node = t_node

    def run(self):
        self.t_node.reset()
        self.finish()

class LookForBall(Node):
    def __init__(self, pan=0, tilt=0, duration=2.0):
        """
        pan: (left/right) in degrees
        tilt: (up/down) in degrees
        duration: time in seconds
        """
        super(LookForBall, self).__init__()
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
        super(DelayTimer, self).__init__()
        self.delay = timer
        self.start_time = self.getTime()

    def reset(self):
        super(DelayTimer, self).reset()
        self.start_time = self.getTime()

    def run(self):
        if self.getTime() - self.start_time > self.delay:
            self.finish()

class StrafeBall(Node):
    def __init__(self):
        super(StrafeBall, self).__init__()

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
        super(StrafeBall, self).reset()
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


class Playing(LoopingStateMachine):
    def setup(self):
        blocker = BlockKeeper()
        blocks = {"left": BlockLeftAndStand(time=6.0),#BlockLeftStand(),
                  "right": BlockRightAndStand(time=6.0),#BlockRightStand(),
                  "center": pose.Sit()
                  }
        stand = Stand()
        strafe = StrafeBall()
        center = HeadPos(0, 0)
        look_for_ball = LookForBall()
        delay_timer = DelayTimer(0.5)

        self.add_transition(stand, C, strafe, S('ball'), look_for_ball, C, strafe)
        self.add_transition(blocker, S('ball'), look_for_ball)
        self.add_transition(blocker, S('strafe'), strafe)
        for name in blocks:
            b = blocks[name]
            self.add_transition(strafe, C, delay_timer, C, blocker, S(name), b, T(10), stand)

        # self.trans(Stand(), C, StrafeBall(), C, Stand(), C)

