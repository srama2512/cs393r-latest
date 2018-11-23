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

TARGET_ENUM = core.WO_BEACON_YELLOW_BLUE

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

    class SearchForTarget(Node):
        def __init__(self, pan=0, tilt=0, duration=2.0):
            """
            pan: (left/right) in degrees
            tilt: (up/down) in degrees
            duration: time in seconds
            """
            super(Playing.SearchForTarget, self).__init__()
            self.pan = pan * core.DEG_T_RAD
            self.tilt = tilt
            self.duration = duration 
        def run(self):
            """If the target was seen, then move head towards the target"""
            target = memory.world_objects.getObjPtr(TARGET_ENUM)
            if target.seen:
                print ('Found target')
                sys.stdout.flush()
                self.finish()
            if self.getTime() > self.duration:
                print ('Finished search')
                sys.stdout.flush()
                self.postFailure()

            commands.setHeadPanTilt(pan=self.pan, tilt=self.tilt, time=self.duration)

    class TraversePath(Node):
        def __init__(self, planner_type='geo', delta_pan=6):
            super(Playing.TraversePath, self).__init__()
            if planner_type == 'geo':
                self.planner = GeometricPathPlanner()
            elif planner_type == 'apm':
                self.planner = PotentialPathPlanner()
            else:
                raise NotImplementedError('Unsupported path planner type')
            self.obstacles_ = []
            self.target_ = None
            self.midX = 160
            self.midY = 120
            self.thresh = 20
            self.delta_pan = delta_pan * core.DEG_T_RAD

            self.pid_position = PIDPosition()
            self.pid_position.max_t_res = 0.2
            self.pid_position.max_x_res = 0.5
            self.pid_position.set_const_x(1.2e-3, 0., 0.)
            self.pid_position.set_const_y(4.5e-3, 1e-4, 0., 1000.0)
            self.pid_position.set_const_t(1.2, 0., 0.)

            self.last_seen = self.getTime()

            # self.ewma_pd = self.EWMA(0., 0.95)
            # self.ewma_pb = self.EWMA(0., 0.95)

        # TODO add reset()

        def _get_egocentric(self, dist, bearing):
            return dist * math.cos(bearing), dist * math.sin(bearing)

        def _get_field(self):
            target = memory.world_objects.getObjPtr(TARGET_ENUM)
            opponents = [memory.world_objects.getObjPtr(core.WO_OPPONENT1 + i) for i in range(0, 5)]
            opponents = [op for op in opponents if op.seen == True]
            if target.seen:
                self._target = Point2D(*self._get_egocentric(target.visionDistance, target.visionBearing))
                self.targetX = target.imageCenterX
                self.targetY = target.imageCenterY
            else:
                self._target = None
                self.targetX = None
                self.targetY = None

            self._obstacles = [Obstacle(*self._get_egocentric(obs.visionDistance, obs.visionBearing), r=200.0) for obs in opponents]
            
            if self._target is not None and len([o for o in self._obstacles if o._is_inside(self._target)]) > 0:
                print ('Warning: target inside obstacle')

        def _print_path(self, path):
            if path is None:
                print ('Target is not visible')
                return

            print('Obstacles:')
            print('\n'.join([obs.pos.__str__() for obs in self._obstacles]))
            print('')
            print('Target: ' + self._target.__str__())
            print('Path:')
            print ([(pt1.__str__(), pt2.__str__(), etype) for pt1, pt2, etype in path])
            print('')
            print('')

        def clip(self, x, xmin, xmax):
            return max(min(x, xmax), xmin)

        def run(self):
            self._get_field()
            if self._target is not None:
                pan = core.joint_values[core.HeadYaw]
                if abs(self.targetX - self.midX) > self.thresh:
                   if self.targetX > self.midX:
                       pan -= self.delta_pan
                   else:
                       pan += self.delta_pan
                
                pan = self.clip(pan, -75.0*core.DEG_T_RAD, 75.0*core.DEG_T_RAD)
                path = self.planner.update(self._obstacles, self._target)
                self.last_seen = self.getTime()
            else:
                pan = core.joint_values[core.HeadYaw]
                path = None

                if self.getTime() - self.last_seen > 2.0:
                    commands.setWalkVelocity(0.0, 0.0, 0.0)
                    self.postFailure()

            self._print_path(path)

            if path is None:
                vel_x, vel_y, vel_t = 0, 0, 0
            else:
                pt = path[0][1]
                tx, ty = pt.x, pt.y
                d = math.sqrt(tx ** 2 + ty ** 2)
                theta = math.atan2(ty, tx)

                (vel_x, vel_y, vel_t) = self.pid_position.update((0, 0, 0), (d, 0, theta))

            commands.setWalkVelocity(vel_x, vel_y, vel_t)
            commands.setHeadPanTilt(pan=pan, tilt=-20., time=0.1)


    def setup(self):
        traverse_path = self.TraversePath()
        stand = self.Stand()
        center = self.HeadPos(0, 0)
        off = self.Off()
        leftSearch = self.SearchForTarget(75, -22, 2.0)
        rightSearch = self.SearchForTarget(-75, -22, 2.0)

        # self.trans(stand, C, center, T(4.0), traverse_path, C, pose.Sit(), C, off)

        self.add_transition(stand, C, center)
        self.add_transition(center, T(2.0), leftSearch)
        self.add_transition(leftSearch, C, rightSearch)
        self.add_transition(leftSearch, F, rightSearch)
        self.add_transition(rightSearch, C, traverse_path)
        self.add_transition(rightSearch, F, center)
        self.add_transition(traverse_path, C, pose.Sit(), C, off)
        self.add_transition(traverse_path, F, center)
