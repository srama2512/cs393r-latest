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

    class WalkToBeacon(Node):
        def __init__(self, dimA=3, dimS=3):
            super(Playing.WalkToBeacon, self).__init__()
            self.startTime = self.getTime()
            self.lastActionTime = self.getTime()
            self.lastAction = None
            self.velx = random.uniform(0.2, 0.7)
            self.asami = ASAMI(dimA=dimA, dimS=dimS)
            self.logger = open('logger.txt', 'w')
            self.logger.write('==== Logging ASAMI ====\n')
            self.logger.write('t, Ws_t, Wa_t, gtDistance\n')
            self.smLogger = open('smLogger.txt', 'w')
            self.smLogger.write(str(dimS) + '\n')
            self.amLogger = open('amLogger.txt', 'w')
            self.amLogger.write(str(dimA) + '\n')
            self.currentPhase = 1
            self.delta_theta = 0.1
            self.theta_thresh = 0.1
            self.beacon_bearing = EWMA(0.0, 0.9)

        def run(self):

            beacon = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK]
            obsHeight = None
            gtDistance = None
            boundFlag = 0
            theta_vel = 0.0
            if beacon.seen:
                # print('beacon positions: ', beacon.visionDistance, beacon.visionBearing)
                self.beacon_bearing.update(beacon.visionBearing)
                print('====> WalkToBeacon: Orig Beacon bearing: {:.3f}, Smoothed Beacon bearing: {:.3f}'.format(beacon.visionBearing, self.beacon_bearing.get()))

                if beacon.visionDistance < 500:
                    boundFlag = -1
                elif beacon.visionDistance > 3000:
                    boundFlag = 1

                obsHeight = beacon.radius 
                gtDistance = beacon.visionDistance # comment this line for running on simulator
                # gtDistance = beacon.height # comment this line for running on robot 

            if self.beacon_bearing.get() < -self.theta_thresh:
                theta_vel = -self.delta_theta
            elif self.beacon_bearing.get() > self.theta_thresh:
                theta_vel = self.delta_theta

            control = self.velx
            # gtVelx = getGTVelocities(self.velx, 0, 0)[0]
            print("Control: {:.3f}, obsHeight: {}, gtDistance: {}".format(self.velx, obsHeight, gtDistance))
            # print("Control: {:.3f}, gtVelx: {:.3f}, obsHeight: {}, gtDistance: {}".format(self.velx, gtVelx, obsHeight, gtDistance))
            if self.lastAction is not None:
                self.asami.update(self.lastAction, self.getTime()-self.lastActionTime, obsHeight)
            
            if beacon.seen:
                self.logger.write('{:.3f},{:.3f},{:.3f},{:.3f}\n'.format(self.getTime()-self.startTime, self.asami.Ws_t, self.asami.Wa_t, gtDistance))
                sensorParams = str(self.asami.St.alpha) + ',' + ','.join(map(str, self.asami.St.beta))
                self.smLogger.write('{}\n'.format(sensorParams))
                actionParams = str(self.asami.At.alpha) + ',' + ','.join(map(str, self.asami.At.beta))
                self.amLogger.write('{}\n'.format(actionParams))
            
            if self.getTime() - self.lastActionTime > 3.0 or boundFlag != 0:
                if boundFlag == -1:
                    self.currentPhase = -1
                elif boundFlag == 1:
                    self.currentPhase = 1
                self.velx = random.uniform(0.2, 0.7) * self.currentPhase
                self.lastAction = self.velx
                self.lastActionTime = self.getTime()
            commands.setWalkVelocity(self.velx, 0.0, theta_vel)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    class MeasureObservations(Node):
        def __init__(self):
            super(Playing.MeasureObservations, self).__init__()
            self.reachedBackEnd = False
            self.logger = open('gtSensor.txt', 'w')

        def run(self):
            beacon = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK]
            if beacon.seen:
                if beacon.height >= 2500:
                    self.reachedBackEnd = True
                if beacon.height < 800:
                    self.finish()
                if self.reachedBackEnd:
                    if beacon.height % 100 < 5:
                        self.logger.write('{:.3f},{:.3f}\n'.format(beacon.radius, beacon.height))

            if not self.reachedBackEnd:
                commands.setWalkVelocity(-1.0, 0.0, 0.0)
            else:
                commands.setWalkVelocity(0.5, 0.0, 0.0)

    def setup(self):
        stand = self.Stand()
        walk = self.Walk(0.5)
        walk2 = self.Walk(1.0)
        walk_to_beacon = self.WalkToBeacon()
        sit = pose.Sit()
        off = self.Off()
        center = self.HeadPos(0, 0)
        sensorGT = self.MeasureObservations()
        #self.trans(stand, C)
        # self.trans(stand, C, walk, T(5.0), sit, C, off)
        self.trans(stand, C, center, T(2.0), walk_to_beacon, C, sit, C, off)
        # self.trans(stand, C, center, T(2.0), sensorGT, C, sit, C, off)
