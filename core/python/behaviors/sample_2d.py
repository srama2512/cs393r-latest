"""Sample behavior."""

from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import pdb
import pose
import core
import math
import random
import memory
import commands
import cfgstiff
import mem_objects
import numpy as np

from task import Task
from behaviors.asami import ASAMI
from collections import namedtuple
from state_machine import Node, C, T, StateMachine

observationTuple = namedtuple('observationTuple', ['height', 'bearing', 'beacon_id', 'command', 'dt'])

FIELD_Y = 2000
FIELD_X = 3000

def _norm_angle(theta):
    return math.atan2(math.sin(theta), math.cos(theta))

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
            self.beaconList = [core.WO_BEACON_BLUE_YELLOW,
                               core.WO_BEACON_YELLOW_BLUE,
                               core.WO_BEACON_BLUE_PINK,
                               core.WO_BEACON_PINK_BLUE,
                               core.WO_BEACON_PINK_YELLOW,
                               core.WO_BEACON_YELLOW_PINK]
            count = 0
            self.b_angles = [0, math.pi, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, math.pi*3./4., -math.pi*3./4.]
            self.a_vels = [-1./2., -1./6., 0., 1./6., 1./2.]

            for a in self.a_vels:
                for b in self.b_angles:
                    magn = math.sqrt(1 - a**2)
                    vx = magn*math.cos(b)
                    vy = magn*math.sin(b)
                    self.control_to_action[count] = (vx, vy, a)
                    count += 1

            self.startTime = self.getTime()
            self.lastFrameTime = None
            # self.lastBeaconSeenTime = self.getTime()
            self.lastControlTime = None
            self.lastControl = None
            self.logger = open('2d_asami_data.txt', 'w')
            self.logger.write('ht, bear, bid, cmd, dt, dist\n')
            self.logger.flush()
            self.gtStateLogger = open('state_log.txt', 'w')
            self.gtStateLogger.write('X, Y, theta\n')
            self.gtStateLogger.flush()
            # self.control_sequence = list(range(len(self.control_to_action)))
            # self.generate_control_sequence()
            self.status = 0
            self.scanningLeft = True
            # self.headPanThres = 70.0 * core.DEG_T_RAD
            self.headPanThres = 60.0 * core.DEG_T_RAD

        def generate_control_sequence(self):
            a_idxs = list(range(len(self.a_vels)))
            random.shuffle(a_idxs)
            b_idxs = list(range(len(self.b_angles)//2))
            random.shuffle(b_idxs)
            self.control_sequence = []
            for i in a_idxs:
                for j in b_idxs:
                    self.control_sequence.append(i*len(self.b_angles) + j*2)
                    self.control_sequence.append(i*len(self.b_angles) + j*2+1)


        def run(self):

            beacons = map(lambda x: mem_objects.world_objects[x], self.beaconList)
            playerObj = mem_objects.memory.world_objects.getObjPtr(mem_objects.memory.robot_state.WO_SELF)
            # playerObj =  mem_objects.world_objects[core.WO_TEAM5]
            locX = playerObj.loc.x
            locY = playerObj.loc.y
            locTheta = playerObj.orientation
            print('locX: {}, locY: {}, locTheta: {}'.format(locX, locY, locTheta))
            # ['height', 'bearing', 'beacon_id', 'command', 'dt']
            height = -1000
            bearing = -1000
            beacon_id = -1000
            distance = -1000
            if self.lastFrameTime is not None:
                bs = list(enumerate(beacons))
                random.shuffle(bs)
                for bid, beacon in bs:
                    if beacon.seen:
                        # print('beacon positions: ', beacon.visionDistance, beacon.visionBearing)
                        height = beacon.radius
                        distance = beacon.visionDistance
                        bearing = beacon.visionBearing
                        beacon_id = bid
                        self.lastBeaconSeenTime = self.getTime()
                        break

                command = self.lastControl
                dt = self.getTime() - self.lastFrameTime
                self.logger.write('{}, {}, {}, {}, {}, {}\n'.format(height, bearing, beacon_id, command, dt, distance))
                self.gtStateLogger.write('{}, {}, {}\n'.format(locX, locY, _norm_angle(locTheta)))

            if self.lastControl is None or self.getTime() - self.lastControlTime > 3.0:
                self.lastControl = random.randint(0, len(self.control_to_action)-1)
                self.lastControlTime = self.getTime()

                # self.lastControl = self.control_sequence[self.status]
                # self.status = (self.status + 1)%len(self.control_to_action)
                # self.lastControlTime = self.getTime()

            # nuclear option
            if abs(locX) > FIELD_X * 0.4 or abs(locY) > FIELD_Y * 0.4:
                b_to_origin = _norm_angle(math.atan2(locY, locX) + math.pi - locTheta)
                self.lastControl = np.argmin((b_to_origin - np.array(self.b_angles))**2) + len(self.b_angles)*2
                self.lastControlTime = self.getTime()

            # Walk backward
            if core.sensor_values[core.headRear] > 0.5:
                self.lastControl = random.choice([1, 9, 17, 25, 33])
                self.lastControlTime = self.getTime()
            # Walk forward
            elif core.sensor_values[core.headFront] > 0.5:
                self.lastControl = random.choice([1, 9, 17, 25, 33])-1
                self.lastControlTime = self.getTime()

            action = self.control_to_action[self.lastControl]
            self.lastFrameTime = self.getTime()
            commands.setWalkVelocity(*action)

            pan = core.joint_values[core.HeadYaw]
            if pan > self.headPanThres:
                self.scanningLeft = False
            elif pan < -self.headPanThres:
                self.scanningLeft = True

            delta = 30 * core.DEG_T_RAD
            if self.scanningLeft:
                commands.setHeadPan(delta, 0.5, True)
            else:
                commands.setHeadPan(-delta, 0.5, True)



    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    class PrintSensors(Node):
        def run(self):
            print('headFront: {:15.5f}, headMiddle: {:15.5f}, headRead: {:15.5f}'.format(core.sensor_values[core.headFront], 
                                                                                         core.sensor_values[core.headMiddle],
                                                                                         core.sensor_values[core.headRear]))

    def setup(self):
        stand = self.Stand()
        walk = self.Walk(0.5)
        walk2 = self.Walk(1.0)
        explore = self.Explore()
        sit = pose.Sit()
        off = self.Off()
        center = self.HeadPos(0, 0)
        print_sensors = self.PrintSensors()

        self.trans(stand, C, center, T(2.0), explore, C, sit, C, off)
        # self.trans(stand, C, print_sensors, T(500.0), sit, C, off)