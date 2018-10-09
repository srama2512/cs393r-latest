"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
from state_machine import Node, S, T, F, C, LoopingStateMachine
import UTdebug
import math
import memory
from memory import joint_commands
from pose import BlockLeftStand, BlockRightStand, BlockCenterStand

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
        self.beta = 0.0
        self.delta_time = 0.3
        self.y_dist_thresh = 150

    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL_KF]
        # commands.setHeadPan(ball.bearing, 0.1)
        commands.setHeadPanTilt(pan=0.0, tilt=0.0, time=2.0)
        if ball.seen:
            ball_x = ball.distance * math.cos(ball.bearing)
            ball_y = ball.distance * math.sin(ball.bearing)
            print('===> Keeper: Ball seen: ball_x: {:.3f}, ball_y: {:.3f}  vel_x: {:.3f}  vel_y: {:.3f}'.format(ball_x, ball_y, self.vel_x, self.vel_y))

            duration = self.getTime() - self.prev_time + 1e-5

            vel_x = (ball_x - self.prev_x)/duration
            vel_y = (ball_y - self.prev_y)/duration

            self.vel_x = self.beta * self.vel_x + (1-self.beta) * vel_x
            self.vel_y = self.beta * self.vel_y + (1-self.beta) * vel_y

            self.prev_time = self.getTime()
            self.prev_x = ball_x
            self.prev_y = ball_y

            if ball_x + self.vel_x * self.delta_time < 0.0:
                
                time_to_reach = (0 - ball_x) / (self.vel_x + 1e-5)
                print("===> Ball is close, blocking!  vel_x: {:.3f}  vel_y: {:.3f}".format(self.vel_x, self.vel_y))
                y_pred = time_to_reach * self.vel_y + ball_y
                print('Predicted y at end: {}'.format(y_pred))

                if y_pred > self.y_dist_thresh:
                    choice = "left"
                elif y_pred < -self.y_dist_thresh:
                    choice = "right"
                else:
                    choice = "center"
                self.postSignal(choice)

class ResetNode(Node):
    def __init__(self, t_node):
        super(ResetNode, self).__init__()
        self.t_node = t_node

    def run(self):
        self.t_node.reset()
        self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": BlockLeftStand(),
                  "right": BlockRightStand(),
                  "center": BlockCenterStand()
                  }
        stand = Stand()
        center = HeadPos(0, 0)
        for name in blocks:
            b = blocks[name]
            self.add_transition(stand, C, blocker, S(name), b, T(1), ResetNode(b), T(1), blocker, C, stand)
