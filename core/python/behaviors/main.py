"""Blank behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

from task import Task

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

    class ReadJoints(Node):
        def read_joints(self):
            joints = {}
            joints['HeadYaw'] = core.joint_values[core.HeadYaw]
            joints['HeadPitch'] = core.joint_values[core.HeadPitch]
            joints['LHipYawPitch'] = core.joint_values[core.LHipYawPitch]
            joints['LHipRoll'] = core.joint_values[core.LHipRoll]
            joints['LHipPitch'] = core.joint_values[core.LHipPitch]
            joints['LKneePitch'] = core.joint_values[core.LKneePitch]
            joints['LAnklePitch'] = core.joint_values[core.LAnklePitch]
            joints['LAnkleRoll'] = core.joint_values[core.LAnklePitch]
            joints['RHipYawPitch'] = core.joint_values[core.RHipYawPitch]
            joints['RHipRoll'] = core.joint_values[core.RHipRoll]
            joints['RHipPitch'] = core.joint_values[core.RHipPitch]
            joints['RKneePitch'] = core.joint_values[core.RKneePitch]
            joints['RAnklePitch'] = core.joint_values[core.RAnklePitch]
            joints['RAnkleRoll'] = core.joint_values[core.RAnkleRoll]
            joints['LShoulderPitch'] = core.joint_values[core.LShoulderPitch]
            joints['LShoulderRoll'] = core.joint_values[core.LShoulderRoll]
            joints['LElbowYaw'] = core.joint_values[core.LElbowYaw]
            joints['LElbowRoll'] = core.joint_values[core.LElbowRoll]
            joints['RShoulderPitch'] = core.joint_values[core.RShoulderPitch]
            joints['RShoulderRoll'] = core.joint_values[core.RShoulderRoll]
            joints['RElbowYaw'] = core.joint_values[core.RElbowYaw]
            joints['RElbowRoll'] = core.joint_values[core.RElbowRoll]
            print('==== Joint sensor readings ====')
            for k, v in joints.iteritems():
                print('{:>15s}: {:.5f}'.format(k, v))

        def run(self):
            self.read_joints()
            self.finish()

    class ReadPressureSensors(Node):
        def read_pressure(self):
            pressure_sensors = {}
            pressure_sensors['fsrLFL'] = core.sensor_values[core.fsrLFL]
            pressure_sensors['fsrLFR'] = core.sensor_values[core.fsrLFR]
            pressure_sensors['fsrLRL'] = core.sensor_values[core.fsrLRL]
            pressure_sensors['fsrLRR'] = core.sensor_values[core.fsrLRR]
            pressure_sensors['fsrRFL'] = core.sensor_values[core.fsrRFL]
            pressure_sensors['fsrRFR'] = core.sensor_values[core.fsrRFR]
            pressure_sensors['fsrRRL'] = core.sensor_values[core.fsrRRL]
            pressure_sensors['fsrRRR'] = core.sensor_values[core.fsrRRR]
            pressure_sensors['bumperLL'] = core.sensor_values[core.bumperLL]
            pressure_sensors['bumperLR'] = core.sensor_values[core.bumperLR]
            pressure_sensors['bumperRL'] = core.sensor_values[core.bumperRL]
            pressure_sensors['bumperRR'] = core.sensor_values[core.bumperRR]
            pressure_sensors['centerButton'] = core.sensor_values[core.centerButton]
            pressure_sensors['headFront'] = core.sensor_values[core.headFront]
            pressure_sensors['headMiddle'] = core.sensor_values[core.headMiddle]
            pressure_sensors['headRear'] = core.sensor_values[core.headRear]
            print('==== Pressure sensor readings ====')
            for k, v in pressure_sensors.iteritems():
                print('{:>15s}: {:.5f}'.format(k, v))
        
        def run(self):
            self.read_pressure()
            self.finish()

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
        def __init__(self, duration=3.0):
            """
            duration: time in seconds
            """
            super(Playing.LookAtBall, self).__init__()
            self.duration = duration
            self.thresh = 20 # if the ball is 5 pixels away, only then move
            self.delta_pan = 6 * core.DEG_T_RAD
            self.delta_tilt = 10 * core.DEG_T_RAD
            self.midX = 640
            self.midY = 480
            self.ball_seen = False

        def run(self):
            """If the ball was seen, then move head towards the ball"""
            ball = memory.world_objects.getObjPtr(core.WO_BALL)

            # if self.ball_seen != ball.seen:
            #     if ball.seen:
            #         core.ledsC.allRightEye(1, 1, 1)
            #     else:
            #         core.ledsC.allRightEye(0, 0, 0)
            #     self.ball_seen = ball.seen

            if ball.seen:
                ballX = ball.imageCenterX
                ballY = ball.imageCenterY
                print('vertY: ', ballY - self.midY)
                if abs(ballY - self.midY) > self.thresh:
                    if ballY > self.midY:
                        tilt = core.joint_values[core.HeadPitch] - self.delta_tilt
                    else:
                        tilt = core.joint_values[core.HeadPitch] + self.delta_tilt
                    commands.setHeadTilt(tilt * core.RAD_T_DEG, target_time=self.duration)
                
                print('vertX: ', ballX - self.midX)
                if abs(ballX - self.midX) > self.thresh:
                   if ballX > self.midX:
                       pan = core.joint_values[core.HeadYaw] - self.delta_pan
                   else:
                       pan = core.joint_values[core.HeadYaw] + self.delta_pan
                   commands.setHeadPan(pan, target_time=self.duration)

    class MaintainDistance(Node):
        def __init__(self, object_type=core.WO_BALL):
            super(Playing.MaintainDistance, self).__init__()

            self.minDist = 750
            self.maxDist = 1000
            self.midX = 640
            self.midY = 480
            self.thresh = 10
            self.turn_angle = 0.1
            self.object_type = object_type

        def run(self):
            """If the ball was seen, then move head towards the ball"""
            ball = memory.world_objects.getObjPtr(self.object_type)
            angle = 0.0

            if ball.seen:

                ballX = ball.imageCenterX
                ballY = ball.imageCenterY

                if abs(ballX - self.midX) > self.thresh:
                   if ballX > self.midX:
                        angle = -1 * self.turn_angle
                   else:
                        angle = self.turn_angle

                ball_dist = ball.visionDistance
                
                if ball_dist < self.minDist:
                   commands.setWalkVelocity(-0.3, 0.0, angle)

                elif ball_dist > self.maxDist:
                    commands.setWalkVelocity(0.3, 0.0, angle)

                else:
                    commands.setWalkVelocity(0.0, 0.0, 0.0)
                

    def setup(self):
        stand = self.Stand()
        walk = self.Walk()
        walkturn = self.WalkTurn()
        walkcurve = self.WalkCurve()
        sit = pose.Sit()
        off = self.Off()
        readjoints = self.ReadJoints()
        readsensors = self.ReadPressureSensors()
        lookatball = self.LookAtBall(0.5)

        maintaindist = self.MaintainDistance(object_type=core.WO_OPP_GOAL)

        center = self.HeadPos(0, 0)
        left = self.HeadPos(22, 0)
        right = self.HeadPos(-22, 0)
        up = self.HeadPos(0, 22, 4.0)
        down = self.HeadPos(0, -22, 4.0)
        
        self.trans(sit, C, center, T(2.0))
        # self.trans(stand, C, readsensors, C, readjoints, C, off, C)
        # self.trans(stand, C, center, T(2.0), left, T(2.0), right, T(2.0), up, T(2.0), down, T(2.0), sit, C, off)
        # self.trans(stand, C, lookatball, C, sit, C, off)
        # self.trans(stand, C, walk, T(5.0), walkturn, T(5.0), walkcurve, T(10.0), sit, C, off)
        # self.trans(stand, C, down, T(2.0), maintaindist, C, sit, C, off)

        # self.trans(stand, C, walkturn, T(5.0), sit, C, off)
        # self.trans(stand, C, sit, C, readjoints, C, off, C, headturn)
