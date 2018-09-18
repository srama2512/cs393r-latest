"""Blank behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

from task import Task

import sys
import math
import time
import core
import pose
import memory
import commands
import cfgstiff
from memory import walk_request, joint_commands
from state_machine import Node, C, T, StateMachine

class Playing(StateMachine):
    """Main behavior task."""

    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()

    class Walk(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0)

    class WalkTurn(Node):
        def run(self):
            commands.setWalkVelocity(0.0, 0, -0.6)

    class WalkCurve(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0.0, 0.2)

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
    
    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    class LookAtBall(Node):
        def __init__(self, delta_pan=6, delta_tilt=10, duration=3.0):
            """
            duration: time in seconds
            """
            super(Playing.LookAtBall, self).__init__()
            self.duration = duration
            self.thresh = 20 # if the ball is 5 pixels away, only then move
            self.delta_pan = delta_pan * core.DEG_T_RAD
            self.delta_tilt = delta_tilt * core.DEG_T_RAD
            self.midX = 160
            self.midY = 120
            self.ball_seen = False

            # Velocity prediction
            self.prev_time = 0.0
            self.prevX = 0.0
            self.prevY = 0.0
            self.velX = 0.0
            self.velY = 0.0
            self.momentum = 0.9
            self.max_vel = 3.0
            self.time_ball_not_present_thresh = 2.0

        def sgn(self, x):
            return 1.0 if x > 0.0 else -1.0

        def run(self):
            """If the ball was seen, then move head towards the ball"""
            ball = memory.world_objects.getObjPtr(core.WO_BALL)

            print ("")
            print (self.velX, self.velY)
            sys.stdout.flush()

            if ball.seen:
                ballX = ball.imageCenterX
                ballY = ball.imageCenterY

                time_diff = (self.getTime() - self.prev_time) + 1e-1
                vX = (ballX - self.prevX) / time_diff
                vY = (ballY - self.prevY) / time_diff

                print ("Raw vels: ", vX, vY)

                self.velX = self.velX * self.momentum + (1. - self.momentum) * vX
                self.velY = self.velY * self.momentum + (1. - self.momentum) * vY

                self.velX = self.velX if abs(self.velX) < self.max_vel else self.sgn(self.velX) * self.max_vel
                self.velY = self.velY if abs(self.velY) < self.max_vel else self.sgn(self.velY) * self.max_vel

                self.prev_time = self.getTime()
                self.prevX = ballX
                self.prevY = ballY

            else:
                time_diff = (self.getTime() - self.prev_time) + 1e-1

                if time_diff > self.time_ball_not_present_thresh:
                    ballX = self.midX
                    ballY = self.midY
                else:
                    ballX = self.prevX + time_diff * self.velX
                    ballY = self.prevY + time_diff * self.velY

            if abs(ballY - self.midY) > self.thresh:
                if ballY > self.midY and core.joint_values[core.HeadPitch] > -22 * core.DEG_T_RAD:
                    tilt = core.joint_values[core.HeadPitch] - self.delta_tilt
                elif core.joint_values[core.HeadPitch] < 0.0:
                    tilt = core.joint_values[core.HeadPitch] + self.delta_tilt
                else:
                    tilt = core.joint_values[core.HeadPitch]
                commands.setHeadTilt(tilt * core.RAD_T_DEG, target_time=self.duration)

            if abs(ballX - self.midX) > self.thresh:
               if ballX > self.midX:
                   pan = core.joint_values[core.HeadYaw] - self.delta_pan
               else:
                   pan = core.joint_values[core.HeadYaw] + self.delta_pan
               commands.setHeadPan(pan, target_time=self.duration)


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


    def setup(self):
        stand = self.Stand()
        walk = self.Walk()
        walkturn = self.WalkTurn()
        walkcurve = self.WalkCurve()
        sit = pose.Sit()
        off = self.Off()
        lookatball = self.LookAtBall(delta_pan=8, duration=0.2)

        center = self.HeadPos(0, 0)

        leftSearch = self.SearchForBall(75, 0, 4.0)
        rightSearch = self.SearchForBall(-75, 0, 4.0)
        downSearch = self.SearchForBall(0, -30, 4.0)

        self.trans(stand, C, center, T(4.0), leftSearch, C, rightSearch, C, downSearch, C, lookatball, C, sit, C, off)

