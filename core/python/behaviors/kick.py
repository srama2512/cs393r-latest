"""Simple behavior that stands, kicks, and then sits down."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import sys
import math
import core
import memory
import pose
import commands
import cfgstiff
from state_machine import StateMachine, Node, C, T, F, LoopingStateMachine
from pid_controller import *

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
                    memory.walk_request.noWalk()
                    self.finish()
                else:
                    print('===> WalkToTarget: visionDistance: {}   visionBearing: {}'.format(ball.visionDistance, ball.visionBearing))
                    memory.walk_request.setWalkTarget(ball.visionDistance + self.d_target_compensation, 0.0, ball.visionBearing, False)
                    print('Calling walk to target')
                    sys.stdout.flush()
                self.last_seen = self.getTime()
            else:
                if self.getTime() - self.last_seen > 2.0:
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
        def __init__(self, goal_b_threshold=0.05, ball_x_threshold=130.0, ball_y_offset=-40.0, ball_y_threshold=10.0, vel_x=0.6, vel_y=0.3, omega=0.1):
            super(Playing.PositionToKick, self).__init__()
            self.goal_b_threshold = goal_b_threshold
            self.ball_x_threshold = ball_x_threshold
            self.ball_y_offset = ball_y_offset
            self.ball_y_threshold = ball_y_threshold
            self.vel_x = vel_x
            self.vel_y = vel_y
            self.omega = omega
            self.last_seen = 0
            self.ptk_ready = False
            self.last_ptk_ready = 0.0

            self.forward_compensation = 100.0
            self.pid_position = PIDPosition()
            self.pid_position.set_const_x(1.2e-3, 0., 0.)
            self.pid_position.set_const_y(3.0e-3, 1e-4, 0., 1000.0)
            self.pid_position.set_const_t(1.2, 0., 0.)

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

                if abs(goal.visionBearing) < self.goal_b_threshold and abs(ball_side_dist - self.ball_y_offset) < self.ball_y_threshold and abs(ball_fwd_dist) < self.ball_x_threshold:
                    print('===> PositionToKick: Reached within bearing of the goal and distance of the ball!')
                    print('===> PositionToKick: FINAL - visionDistanceGoal: {}   visionBearingGoal: {}   ball_side_dist: {}   ball_fwd_dist: {}'.format(goal.visionDistance, 
                                                                                                                                               goal.visionBearing, 
                                                                                                                                               ball_side_dist,
                                                                                                                                               ball_fwd_dist))
                    commands.setWalkVelocity(0.0, 0.0, 0.0)
                    self.finish()
                else:
                    (vel_x, vel_y, vel_t) = self.pid_position.update((0, 0, 0), (ball_fwd_dist, ball_side_dist - self.ball_y_offset, goal.visionBearing))
                    print ("===> PositionToKick: vel_x : ", vel_x, "vel_y :", vel_y, "vel_t : ", vel_t)
                    commands.setWalkVelocity(vel_x, vel_y, vel_t)
            else:
                print('===> PositionToKick: Not seeing either the ball (or) the goal')
                if self.getTime() - self.last_seen > 2.0:
                    self.postFailure()

    class Dribble(Node):
        def __init__(self, goal_b_threshold=0.15, goal_x_threshold=1200.0, ball_x_threshold=220.0, ball_y_threshold=50.0, vel_x=0.5, vel_y=0.6, omega=0.1):
            super(Playing.Dribble, self).__init__()
            self.goal_b_threshold = goal_b_threshold
            self.goal_x_threshold = goal_x_threshold
            self.ball_x_threshold = ball_x_threshold
            self.ball_y_threshold = ball_y_threshold
            self.vel_x = vel_x
            self.vel_y = vel_y
            self.omega = omega
            self.last_seen = 0
            self.dribble = False
            self.goal_not_seen = False

        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)

            # if goal.seen and ball.seen:
            if ball.seen:
                ball_side_dist = ball.visionDistance * math.sin(ball.visionBearing)
                ball_fwd_dist = ball.visionDistance * math.cos(ball.visionBearing)
                #sys.stdout.flush()
                if goal.seen:
                    self.goal_not_seen = False
                    goal_vision_bearing = goal.visionBearing
                    goal_fwd_dist = goal.visionDistance * math.cos(goal.visionBearing)
                else:
                    self.goal_not_seen = True
                    goal_vision_bearing = 10.0 # self.goal_b_threshold + 1.0
                    goal_fwd_dist = self.goal_x_threshold + 1.0

                print('===> Dribble: visionDistanceGoal: {}   visionBearingGoal: {}   ball_side_dist: {}   ball_fwd_dist: {}'.format(goal_fwd_dist, 
                                                                                                                                               goal_vision_bearing, 
                                                                                                                                               ball_side_dist,
                                                                                                                                               ball_fwd_dist))
                if abs(goal_vision_bearing) < self.goal_b_threshold and abs(ball_side_dist) < self.ball_y_threshold:
                    print('===> Dribble: Reached within bearing of the goal and distance of the ball!')
                    self.dribble = True

                    if abs(goal_fwd_dist) < self.goal_x_threshold:
                        print('===> Dribble: Reached the goal!')
                        sys.stdout.flush()
                        self.finish()

                if goal_vision_bearing < -self.goal_b_threshold:
                    omega = -self.omega
                elif goal_vision_bearing > self.goal_b_threshold:
                    omega = self.omega
                else:
                    omega = 0.0

                if ball_side_dist < -self.ball_y_threshold:
                    vel_y = -self.vel_y
                elif ball_side_dist > self.ball_y_threshold:
                    vel_y = self.vel_y
                else:
                    vel_y = 0.0

                if self.dribble:
                    vel_x = self.vel_x
                    print('===> Dribble: walking the ball')
                else:
                    if ball_fwd_dist < -self.ball_x_threshold:
                        vel_x = -self.vel_x
                    elif ball_fwd_dist > self.ball_x_threshold:
                        vel_x = self.vel_x
                    else:
                        vel_x = 0.0

                self.last_seen = self.getTime()
                self.dribble = False
                if self.goal_not_seen:
                    omega *= 2
                commands.setWalkVelocity(vel_x, vel_y, omega)

            else:
                print('===> Dribble: Not seeing the ball')
                if self.getTime() - self.last_seen > 2.0:
                    self.postFailure()

    class RotateBody(Node):
        def __init__(self):
            super(Playing.RotateBody, self).__init__()

        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            if ball.seen:
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
        leftSearch = self.SearchForBall(75, -22, 4.0)
        rightSearch = self.SearchForBall(-75, -22, 4.0)
        rotateBody = self.RotateBody()

        # self.trans(stand, C, center, T(4.0), leftSearch, C, rightSearch, C, lookatball, C, walk_to_target, C, ptd, C, self.Stand(), C)
        
        self.add_transition(stand, C, center)
        self.add_transition(center, T(4.0), leftSearch)
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
        self.add_transition(kick, C, stand)

        #self.trans(self.PositionToKick(), C)
        # self.trans(self.Stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
        # self.trans(self.Stand(), C, self.Kick(), C, self.Unstiffen(), C)

        # self.trans(self.Stand(), C, self.Unstiffen(), C)
