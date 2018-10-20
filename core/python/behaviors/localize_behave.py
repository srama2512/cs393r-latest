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
import pose
import cfgstiff

def sgn(x):
    return -1.0 if x < 0 else 1.0

class EWMA():
    # Exponentially weighted moving average
    def __init__(self, val, gamma):
        self.val = val
        self.gamma = gamma

    def update(self, val):
        self.val = self.gamma * self.val + (1-self.gamma) * val

    def get(self):
        return self.val

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

class ResetNode(Node):
    def __init__(self, t_node):
        super(ResetNode, self).__init__()
        self.t_node = t_node

    def run(self):
        self.t_node.reset()
        self.finish()

class Off(Node):
    def run(self):
        commands.setStiffness(cfgstiff.Zero)
        if self.getTime() > 2.0:
            memory.speech.say("turned off stiffness")
            self.finish()

class Walk(Node):
    def run(self):
        commands.setWalkVelocity(0.5, 0, 0)

class WalkTurn(Node):
    def run(self):
        commands.setWalkVelocity(0.0, 0, -0.2)

class WalkCurve(Node):
    def run(self):
        commands.setWalkVelocity(0.5, 0.0, 0.2)

class WalkToCenter(Node):
    def __init__(self, delta_theta=0.1, thresh_theta=0.3):
        super(WalkToCenter, self).__init__()
        self.delta_theta = delta_theta
        self.thresh_theta = thresh_theta
        self.locErrSmooth = EWMA(0.0, 0.9)
        self.xVelSmooth = EWMA(0.0, 0.95)
        self.radiusSmooth = EWMA(0.0, 0.9)
        self.xVelLimit = 1.00
        self.headRotDir = 1.0
        self.headRotDur = 3
        self.headPanLimit = 45.0 * core.DEG_T_RAD
        self.lastShift = self.getTime()
        self.radiusThresh = 100.0

    def run(self):
        robot_state = mem_objects.memory.world_objects.getObjPtr(mem_objects.memory.robot_state.WO_SELF)
        locX, locY = robot_state.loc.x, robot_state.loc.y
        locTheta = robot_state.orientation
        locAlpha = math.atan2(locY, locX)
        # Localization error update
        locErr = locTheta - locAlpha - math.pi
        self.locErrSmooth.update(locErr)
        # Radius update
        radius = math.sqrt(locX ** 2 + locY ** 2)
        self.radiusSmooth.update(radius)
        # xVel update
        xVel = 0.0
        if abs(locErr) < self.thresh_theta:
            xVel = self.xVelLimit
        self.xVelSmooth.update(xVel)
        # Head rotation update
        if self.getTime() - self.lastShift > self.headRotDur:
            self.lastShift = self.getTime()
            self.headRotDir *= -1.0
        # Set rotation direction
        rotDir = -sgn(self.locErrSmooth.get())

        print('===> WTC: x: {:.2f}, y: {:.2f}, theta: {:.2f}, alpha: {:.2f}, error: {:.2f} xVel: {:.2f}, rotVel: {:.2f}, rad: {:.2f}'.format(locX, locY, \
                                                                                locTheta, locAlpha, self.locErrSmooth.get(), self.xVelSmooth.get(), \
                                                                                rotDir * self.delta_theta, self.radiusSmooth.get()))
        
        if self.radiusSmooth.get() < self.radiusThresh:
            commands.setWalkVelocity(0.0, 0.0, 0.0)
            commands.setHeadPanTilt(pan=0.0, tilt=0.0, time=1.0)
            self.finish()
                
        commands.setHeadPanTilt(pan=self.headPanLimit * self.headRotDir, tilt=-10.0, time=self.headRotDur)
        commands.setWalkVelocity(self.xVelSmooth.get(), 0.0, rotDir * self.delta_theta)

class Playing(StateMachine):
    def setup(self):
        center = HeadPos(0, 0)
        sit = pose.Sit()
        self.trans(Stand(), C, WalkToCenter(), C, Stand(), C, sit, C, Off())
