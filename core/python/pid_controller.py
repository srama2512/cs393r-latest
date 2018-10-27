"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import pdb
import core
import commands
import mem_objects
from state_machine import Node, S, T, F, C, LoopingStateMachine, StateMachine
import UTdebug
import math
import memory
from memory import joint_commands
import pose
import cfgstiff


def clip(val, mi, mx):
    return max(min(val, mx), mi)

class PIDController(object):
    def __init__(self, cp=0., ci=0., cd=0.):
        self.cp = cp
        self.ci = ci
        self.cd = cd
        self.curr_error = 0.0
        self.prev_error = 0.0
        self.cumulative_error = 0.0
        self.max_cumul_error = float('inf')

    def set_constants(self, cp, ci, cd, max_cumul_error=float('inf')):
        self.cp = cp
        self.ci = ci
        self.cd = cd
        self.max_cumul_error = max_cumul_error
        self.reset()

    def clip_cumulative(self):
        if abs(self.cumulative_error) > self.max_cumul_error:
            self.cumulative_error = self.max_cumul_error if self.cumulative_error > 0.0 else -self.max_cumul_error

    def update(self, current, target):
        self.prev_error = self.curr_error
        self.curr_error = target - current
        self.cumulative_error += self.curr_error

        self.clip_cumulative()

        return self.cp * self.curr_error + self.ci * self.cumulative_error + \
            self.cd * (self.curr_error - self.prev_error)

    def reset(self):
        self.curr_error = 0.0
        self.prev_error = 0.0
        self.cumulative_error = 0.0
 
class PIDPosition(object):
    def __init__(self):
        self.pid_x = PIDController()
        self.pid_y = PIDController()
        self.pid_t = PIDController()
        self.max_t_res = 1.0

    def set_const_x(self, cp, ci, cd):
        self.pid_x.set_constants(cp, ci, cd)

    def set_const_y(self, cp, ci, cd, max_cumul_error=float('inf')):
        self.pid_y.set_constants(cp, ci, cd, max_cumul_error)

    def set_const_t(self, cp, ci, cd):
        self.pid_t.set_constants(cp, ci, cd)
        
    def update(self, curr, target):
        res_x = self.pid_x.update(curr[0], target[0])
        res_y = self.pid_y.update(curr[1], target[1])
        res_t = self.pid_t.update(curr[2], target[2])

        res_t = clip(res_t, -self.max_t_res, self.max_t_res)

        print ("PIDController: x : ", "curr_error: ", self.pid_x.curr_error, "curr-prev: ", self.pid_x.curr_error-self.pid_x.prev_error, "cumulative_error: ", self.pid_x.cumulative_error)
        print ("PIDController: y : ", "curr_error: ", self.pid_y.curr_error, "curr-prev: ", self.pid_y.curr_error-self.pid_y.prev_error, "cumulative_error: ", self.pid_y.cumulative_error)
        print ("PIDController: t : ", "curr_error: ", self.pid_t.curr_error, "curr-prev: ", self.pid_t.curr_error-self.pid_t.prev_error, "cumulative_error: ", self.pid_t.cumulative_error)

        return (res_x, res_y, res_t)
        