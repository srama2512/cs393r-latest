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
from path import *

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

    class TraversePath(Node):
        def __init__(self, planner_type='geo'):
            if planner_type == 'geo':
                self.planner = GeometricPathPlanner()
            elif planner_type == 'apm':
                self.planner = PotentialPathPlanner()
            else:
                raise NotImplementedError('Unsupported path planner type')
            self.obstacles_ = []
            self.target_ = None

        def _get_egocentric(self, dist, bearing):
            return dist * math.cos(bearing), dist * math.sin(bearing)

        def _get_field(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            opponents = [memory.world_objects.getObjPtr(core.WO_OPPONENT1 + i) for i in range(0, 5)]
            opponents = [op for op in opponents if op.seen == True]

            self._target = Point2D(*self._get_egocentric(ball.visionDistance, ball.visionBearing))
            self._obstacles = [Obstacle(*self._get_egocentric(obs.visionDistance, obs.visionBearing), r=obs.radius)]

        def run(self):
            self._get_field()

            path = self.planner.update(self._obstacles, self._target)

            print (path)

    def setup(self):
        traverse_path = self.TraversePath()
        stand = self.Stand()
        center = self.HeadPos(-22, 0)

        self.trans(stand, C, center, T(4.0), traverse_path, C, pose.Sit(), C, off)
