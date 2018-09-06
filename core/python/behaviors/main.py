from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

from task import Task

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
            commands.setWalkVelocity(0.5, 0, -0.6)

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

    def setup(self):
        stand = self.Stand()
        walk = self.Walk()
        walkturn = self.WalkTurn()
        sit = pose.Sit()
        off = self.Off()
        readjoints = self.ReadJoints()
        readsensors = self.ReadPressureSensors()

        center = self.HeadPos(0, 0)
        left = self.HeadPos(22, 0)
        right = self.HeadPos(-22, 0)
        up = self.HeadPos(0, 22, 4.0)
        down = self.HeadPos(0, -22, 4.0)

        # self.trans(readsensors, C, off, C)
        # self.trans(center, T(2.0), left, T(2.0), right, T(2.0), up, T(2.0), down, T(2.0), off, C)
        self.trans(stand, C, walkturn, T(5.0), sit, C, off)
        # self.trans(stand, C, sit, C, readjoints, C, off, C, headturn)


