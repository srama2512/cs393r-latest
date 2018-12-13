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

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    class StiffenAll(Node):
        def run(self):
            memory.walk_request.noWalk()
            commands.setStiffness(cfgstiff.One)

    class Unstiffen(Node):
        def run(self):
            memory.walk_request.noWalk()
            commands.setStiffness(cfgstiff.RightLegOffRestWalk)

    def setup(self):
        self.trans(self.Stand(), C, self.Unstiffen(), C)