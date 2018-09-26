"""Simple behavior that stands, kicks, and then sits down."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import sys
import core
import memory
import pose
import commands
import cfgstiff
from state_machine import StateMachine, Node, C


class Playing(StateMachine):
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

    class WalkToTarget(Node):
        def __init__(self, d_threshold=50.0):
            super(Playing.WalkToTarget, self).__init__()
            self.d_threshold = d_threshold

        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)

            if ball.seen:
                core.ledsC.frontRightEar(1)
                core.ledsC.backRightEar(1)
                core.ledsC.frontLeftEar(0)
                core.ledsC.backLeftEar(0)

                print('===> WalkToTarget: visionDistanceBall: {}   visionBearingBall: {}'.format(ball.visionDistance, ball.visionBearing))
                memory.walk_request.setWalkTarget(ball.visionDistance, 0.0, ball.visionBearing, False)
                print('===> WalkToTarget: Calling walk to target')
                # sys.stdout.flush()

                if ball.visionDistance < self.d_threshold:
                    print('===> WalkToTarget: Reached within distance of ball!')
                    # sys.stdout.flush()
                    self.finish()
            else:
                print('===> WalkToTarget: Ball not seen :(')
                # sys.stdout.flush()

    class PositionToDribble(Node):

        def __init__(self, goal_b_threshold=0.1, ball_d_threshold=50, vel_x=-0.05, vel_y=0.1, omega=0.1):
            super(Playing.PositionToDribble, self).__init__()
            self.goal_b_threshold = goal_b_threshold
            self.ball_d_threshold = ball_d_threshold
            self.vel_x = vel_x
            self.vel_y = vel_y
            self.omega = omega

        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)

            if goal.seen and ball.seen:

                print('===> PositionToDribble: visionDistanceGoal: {}   visionBearingGoal: {}'.format(goal.visionDistance, goal.visionBearing))
                sys.stdout.flush()

                ball_side_dist = ball.visionDistance * math.sin(ball.visionBearing)

                if abs(goal.visionBearing) < self.goal_b_threshold and ball_side_dist < self.ball_d_threshold:
                    print('===> PositionToDribble: Reached within bearing of the goal and distance of the ball!')
                    sys.stdout.flush()
                    self.finish()

                if goal.visionBearing < -self.goal_b_threshold:
                    omega = -self.omega
                elif goal.visionBearing > self.goal_b_threshold:
                    omega = self.omega
                else:
                    omega = 0.0

                if ball_side_dist < -self.ball_d_threshold:
                    vel_y = -self.ball_d_threshold
                elif ball_side_dist > self.ball_d_threshold:
                    vel_y = self.ball_d_threshold
                else:
                    vel_y = 0.0

                commands.setWalkVelocity(self.vel_x, vel_y, omega)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    def setup(self):
        # self.trans(self.Stand(), C, self.WalkToTarget())#, self.PositionToDribble(), C)
        self.trans(self.Stand(), C, self.WalkToTarget(), C)
        #self.trans(self.Stand(), C, self.Kick(), C, self.Stand(),
        #           C, pose.Sit(), C, self.Off())
