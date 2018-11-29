"""Sample behavior."""

from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import pose
import core
import math
import random
import memory
import commands
import cfgstiff
import mem_objects

from task import Task
from behaviors.asami import ASAMI
from collections import namedtuple
from state_machine import Node, C, T, StateMachine

observationTuple = namedtuple('observationTuple', ['height', 'bearing', 'beacon_id', 'command', 'dt'])

class Ready(Task):
    def run(self):
        commands.standStraight()
        if self.getTime() > 5.0:
            memory.speech.say("ready to play")
            self.finish()

def getGTVelocities(velX, velY, velT):
    return velX*240.0, velY*120.0, velT*math.radians(130.0)

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

class Playing(StateMachine):
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
            super(Playing.HeadPos, self).__init__()
            self.pan = pan * core.DEG_T_RAD
            self.tilt = tilt
            self.duration = duration 
        def run(self):
            commands.setHeadPanTilt(pan=self.pan, tilt=self.tilt, time=self.duration)

    class Walk(Node):
        def __init__(self, vel):
            super(Playing.Walk, self).__init__()
            self.vel = vel
        def run(self):
            commands.setWalkVelocity(self.vel, 0, 0)

    class Explore(Node):
        def __init__(self, dimA=3, dimS=3):
            super(Playing.Explore, self).__init__()
            self.control_to_action = {}
            self.beaconList = [core.WO_BEACON_BLUE_YELLOW
                               core.WO_BEACON_YELLOW_BLUE,
                               core.WO_BEACON_BLUE_PINK
                               core.WO_BEACON_PINK_BLUE
                               core.WO_BEACON_PINK_YELLOW
                               core.WO_BEACON_YELLOW_PINK]
            count = 0
            for a in [-1./2., -1./6., 0., 1./6., 1./2.]:
                for b in [0, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, math.pi*3./4., -math.pi*3./4., math.pi]:
                    magn = math.sqrt(1 - a**2)
                    vx = magn*math.cos(b)
                    vy = magn*math.sin(b)
                    self.control_to_action[count] = (vx, vy, a)
                    count += 1

            self.startTime = self.getTime()
            self.lastFrameTime = None
            self.lastControlTime = None
            self.lastControl = None
            self.logger = open('2d_asami_data.txt', 'w')
            self.logger.write('ht, bear, bid, cmd, dt\n')

        def run(self):

            beacons = map(lambda x: mem_objects.world_objects[x], self.beaconList)

            # ['height', 'bearing', 'beacon_id', 'command', 'dt']
            height = -1000
            bearing = -1000
            beacon_id = -1000
            if self.lastFrameTime is not None:
                bs = enumerate(beacons)
                random.shuffle(bs)
                for bid, beacon in bs:
                    if beacon.seen:
                        # print('beacon positions: ', beacon.visionDistance, beacon.visionBearing)
                        height = beacon.radius
                        bearing = beacon.visionBearing
                        beacon_id = bid
                        break

                command = self.lastControl
                dt = self.getTime() - self.lastFrameTime
                self.logger.write('{}, {}, {}, {}, {}\n'.format(height, bearing, beacon_id, command, dt))

            if self.lastControl is None or self.getTime() - self.lastControlTime > 3.0:
                self.lastControl = random.randint(0, len(self.control_to_action)-1)
                self.lastControlTime = self.getTime()

            action = self.control_to_action[self.lastControl]
            self.lastFrameTime = self.getTime()
            commands.setWalkVelocity(*action)

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
        explore = self.Explore()
        sit = pose.Sit()
        off = self.Off()
        center = self.HeadPos(0, 0)

        self.trans(stand, C, center, T(2.0), explore, C, sit, C, off)
