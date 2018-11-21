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
from state_machine import Node, C, T, StateMachine


class Ready(Task):
    def run(self):
        commands.standStraight()
        if self.getTime() > 5.0:
            memory.speech.say("ready to play")
            self.finish()

def getGTVelocities(velX, velY, velT):
    return velX*240.0, velY*120.0, velT*math.radians(130.0)


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
            self.startTime = self.getTime()
            self.lastActionTime = self.getTime()
            self.lastAction = None
            self.velx = random.uniform(-1, 1)
            self.asami = ASAMI(dimA=4, dimS=3)
            self.logger = open('logger.txt', 'w')
            self.logger.write('==== Logging ASAMI ====\n')
            self.logger.write('t, Ws_t, Wa_t, gtDistance\n')
            self.currentPhase = 1

        def run(self):

            beacon = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK]
            obsHeight = None
            gtDistance = None
            boundFlag = 0
            if beacon.seen:
                # print('beacon positions: ', beacon.visionDistance, beacon.visionBearing)
                if beacon.visionDistance < 1200:
                    boundFlag = -1
                elif beacon.visionDistance > 2500:
                    boundFlag = 1

                obsHeight = beacon.radius 
                gtDistance = beacon.height # comment this line for running on robot 

            control = self.velx
            gtVelx = getGTVelocities(self.velx, 0, 0)[0]
            print("Control: {:.3f}, gtVelx: {:.3f}, obsHeight: {}, gtDistance: {}".format(self.velx, gtVelx, obsHeight, gtDistance))
            if self.lastAction is not None:
                self.asami.update(self.lastAction, self.getTime()-self.lastActionTime, obsHeight)
            
            if beacon.seen:
                self.logger.write('{:.3f},{:.3f},{:.3f},{:.3f}\n'.format(self.getTime()-self.startTime, self.asami.Ws_t, self.asami.Wa_t, gtDistance))
            
            if self.getTime() - self.lastActionTime > 3.0 or boundFlag != 0:
                if boundFlag == -1:
                    self.currentPhase = -1
                elif boundFlag == 1:
                    self.currentPhase = 1
                self.velx = random.uniform(0, 1) * self.currentPhase
                self.lastAction = self.velx
                self.lastActionTime = self.getTime()
                commands.setWalkVelocity(self.velx, 0.0, 0.0)

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
