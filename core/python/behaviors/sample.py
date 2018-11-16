"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import mem_objects
import pose
import core
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, StateMachine


class Ready(Task):
    def run(self):
        commands.standStraight()
        if self.getTime() > 5.0:
            memory.speech.say("ready to play")
            self.finish()


class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()

    class Walk(Node):
        def __init__(self, vel):
            super(Playing.Walk, self).__init__()
            self.vel = vel
        def run(self):
            commands.setWalkVelocity(self.vel, 0, 0)

    class WalkToBeacon(Node):
        def __init__(self):
            super(Playing.WalkToBeacon, self).__init__()
            self.dir = 1.0

        def run(self):
            beacon = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK]
            if beacon.seen:
                print('beacon positions: ', beacon.visionDistance, beacon.visionBearing)
                if beacon.visionDistance < 500:
                    self.dir = -1.0
                elif beacon.visionDistance > 1500:
                    self.dir = 1.0
                commands.setWalkVelocity(self.dir * 0.5, 0.0, 0.0)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    def setup(self):
        stand = self.Stand()
        walk = self.Walk(0.5)
        walk2 = self.Walk(1.0)
        walk_to_beacon = self.WalkToBeacon()
        sit = pose.Sit()
        off = self.Off()
        #self.trans(stand, C)
        # self.trans(stand, C, sit, C, off)
        self.trans(stand, C, walk_to_beacon, C, sit, C, off)
