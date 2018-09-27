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
from state_machine import StateMachine, Node, C, T


class Playing(StateMachine):
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

    class PositionToDribble(Node):
        def __init__(self, goal_b_threshold=0.05, ball_x_threshold=200.0, ball_y_threshold=30.0, vel_x=0.05, vel_y=0.1, omega=0.1):
            super(Playing.PositionToDribble, self).__init__()
            self.goal_b_threshold = goal_b_threshold
            self.ball_x_threshold = ball_x_threshold
            self.ball_y_threshold = ball_y_threshold
            self.vel_x = vel_x
            self.vel_y = vel_y
            self.omega = omega

        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)

            if goal.seen and ball.seen:

                ball_side_dist = ball.visionDistance * math.sin(ball.visionBearing)
                ball_fwd_dist = ball.visionDistance * math.cos(ball.visionBearing)
                print('===> PositionToDribble: visionDistanceGoal: {}   visionBearingGoal: {}   ball_side_dist: {}   ball_fwd_dist: {}'.format(goal.visionDistance, 
                                                                                                                                               goal.visionBearing, 
                                                                                                                                               ball_side_dist,
                                                                                                                                               ball_fwd_dist))
                #sys.stdout.flush()

                if abs(goal.visionBearing) < self.goal_b_threshold and abs(ball_side_dist) < self.ball_y_threshold and abs(ball_fwd_dist) < self.ball_x_threshold:
                    print('===> PositionToDribble: Reached within bearing of the goal and distance of the ball!')
                    sys.stdout.flush()
                    # self.finish()

                if goal.visionBearing < -self.goal_b_threshold:
                    omega = -self.omega
                elif goal.visionBearing > self.goal_b_threshold:
                    omega = self.omega
                else:
                    omega = 0.0

                if ball_side_dist < -self.ball_y_threshold:
                    vel_y = -self.ball_y_threshold
                elif ball_side_dist > self.ball_y_threshold:
                    vel_y = self.ball_y_threshold
                else:
                    vel_y = 0.0

                if ball_fwd_dist < -self.ball_x_threshold:
                    vel_x = -self.ball_x_threshold
                elif ball_fwd_dist > self.ball_x_threshold:
                    vel_x = self.ball_x_threshold
                else:
                    vel_x = 0.0

                commands.setWalkVelocity(vel_x, vel_y, omega)
            else:
                print('===> PositionToDribble: Not seeing either the ball (or) the goal')

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
                self.finish()

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
            else:
                pass
                # TODO: Revert to search


    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    def setup(self):
        walk_to_target = self.WalkToTarget()
        stand = self.Stand()
        ptd = self.PositionToDribble()
        center = self.HeadPos(0, 0)

        lookatball = self.LookAtBall(delta_pan=12, duration=0.2)
        leftSearch = self.SearchForBall(75, 0, 4.0)
        rightSearch = self.SearchForBall(-75, 0, 4.0)

        self.trans(stand, C, center, T(4.0), leftSearch, C, rightSearch, C, lookatball, C, walk_to_target, C, ptd, C, self.Stand(), C)

        #self.trans(self.PositionToDribble(), C)
        #self.trans(self.Stand(), C, self.Kick(), C, self.Stand(),
        #           C, pose.Sit(), C, self.Off())
