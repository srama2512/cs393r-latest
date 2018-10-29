"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
from state_machine import Node, S, T, F, C, LoopingStateMachine, StateMachine
import UTdebug
import math
import memory
from memory import joint_commands
from pose import BlockLeftAndStand, BlockRightAndStand, Squat, BlockCenterStand#BlockLeftStand, BlockRightStand

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

class Blocker(Node):
    def __init__(self):
        super(Blocker, self).__init__()
        self.prev_x = 0
        self.prev_y = 0
        self.prev_time = self.getTime()
        self.vel_x = 0
        self.vel_y = 0
        self.beta_x = 0.0
        self.beta_y = 0.5
        self.delta_time = 1.20
        self.y_dist_thresh = 100
        self.y_max_dist_thresh = 400

    def reset(self):
        super(Blocker, self).reset()

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_time = self.getTime()
        self.vel_x = 0.0
        self.vel_y = 0.0

    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL_KF]
        # commands.setHeadPan(ball.bearing, 0.1)
        commands.setHeadPanTilt(pan=0.0, tilt=0.0, time=2.0)
        if ball.seen:
            ball_x = ball.distance * math.cos(ball.bearing)
            ball_y = ball.distance * math.sin(ball.bearing)

            duration = self.getTime() - self.prev_time + 1e-5
            print('===> Keeper: Ball seen: duration: {:.3f}, prev_x: {:.3f}, prev_y: {:.3f}, ball_x: {:.3f}, ball_y: {:.3f}  vel_x: {:.3f}  vel_y: {:.3f}'.format(\
                                                    duration, self.prev_x, self.prev_y, ball_x, ball_y, self.vel_x, self.vel_y))

            vel_x = (ball_x - self.prev_x) / duration
            vel_y = (ball_y - self.prev_y) / duration

            self.vel_x = self.beta_x * self.vel_x + (1-self.beta_x) * vel_x
            self.vel_y = self.beta_y * self.vel_y + (1-self.beta_y) * vel_y

            self.prev_time = self.getTime()
            self.prev_x = ball_x
            self.prev_y = ball_y

            if ball_x + self.vel_x * self.delta_time < 0.0:
                
                time_to_reach = (0 - ball_x) / (self.vel_x + 1e-5)
                print("===> Ball is close, blocking!  vel_x: {:.3f}  vel_y: {:.3f}".format(self.vel_x, self.vel_y))
                y_pred = time_to_reach * self.vel_y + ball_y
                print('Predicted y at end of {} secs: {}'.format(time_to_reach, y_pred))

                if abs(y_pred) < self.y_max_dist_thresh:

                    if y_pred > self.y_dist_thresh:
                        choice = "left"
                        print ('\n\n\n\n<----------------- LEFT\n\n\n\n')
                    elif y_pred < -self.y_dist_thresh:
                        choice = "right"
                        print ('\n\n\n\n                                      RIGHT ----------------->\n\n\n\n')
                    else:
                        choice = "center"
                        print ('\n\n\n\n<------------------------- CENTER --------------------------->\n\n\n\n')
                    self.postSignal(choice)

class ResetNode(Node):
    def __init__(self, t_node):
        super(ResetNode, self).__init__()
        self.t_node = t_node

    def run(self):
        self.t_node.reset()
        self.finish()

class Playing(StateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": BlockLeftAndStand(time=6.0),#BlockLeftStand(),
                  "right": BlockRightAndStand(time=6.0),#BlockRightStand(),
                  "center": BlockCenterStand(time=3.0)
                  }
        stand = Stand()
        center = HeadPos(0, 0)
        for name in blocks:
            b = blocks[name]
            # self.add_transition(stand, C, blocker, S(name), b, T(1), ResetNode(b), T(1), blocker, C, stand)
            self.add_transition(stand, C, blocker, S(name), b, T(10), blocker, C, stand)

