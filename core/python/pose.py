#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import task, util, state_machine
import commands
import head
from task import Task, MultiTask
import util
import cfgpose, cfgstiff
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles

class ToPose(Task):
  def __init__(self, pose, time = 2.0, reverse=False):
    Task.__init__(self)
    self.pose = pose
    self.time = time
    self.reverse = reverse
  
  def reset(self):
    super(ToPose, self).reset()
    self.first = True

  def run(self):
    if self.first:
      for i in range(2, core.NUM_JOINTS):
        val = util.getPoseJoint(i, self.pose, self.reverse)
        if val != None:
          joint_commands.setJointCommand(i, val * core.DEG_T_RAD)

      joint_commands.send_body_angles_ = True
      joint_commands.body_angle_time_ = self.time * 1000.0
      walk_request.noWalk()
      kick_request.setNoKick()
      self.first = False

    if self.getTime() > self.time:
      self.finish()

  @staticmethod
  def ToPoseTimes(poses, times):
    posetimes = []
    for i in range(len(poses)):
      posetimes.append(poses[i])
      posetimes.append(times[i])
    return posetimes

class ToPoseMoveHead(MultiTask):
  def __init__(self, pose, tilt = 0.0, time = 2.0):
    self.bpose = ToPose(pose = pose, time = time)
    self.mhead = head.MoveHead(tilt = tilt, time = time)
    super(ToPoseMoveHead, self).__init__(self.bpose, self.mhead)

class PoseSequence(Task):
  def __init__(self, *args):
    super(PoseSequence, self).__init__()
    if len(args) % 2 != 0:
      raise Exception("Pose sequence arguments must be (pose, time) pairs.")
    pi, ti = 0, 1
    chain = []
    while ti < len(args):
      pose = args[pi]
      time = args[ti]
      chain += [ToPose(pose = pose, time = time)]
      pi += 2
      ti += 2
    self.setChain(chain)

  def start(self):
    commands.setStiffness()
    super(PoseSequence, self).start()

  @staticmethod
  def ToPoseTimes(poses, times):
    posetimes = []
    for i in range(len(poses)):
      posetimes.append(poses[i])
      posetimes.append(times[i])
    return posetimes

class Sit(Task):
  def __init__(self):
    Task.__init__(self)
    kick_request.setNoKick()
    walk_request.noWalk()
    kick_request.kick_running_ = False
    behavior_mem.keeperDiving = core.Dive.NONE
    self.state = state_machine.SimpleStateMachine('stop', 'checkarms', 'movearms', 'sit', 'relaxknee', 'relaxbody', 'finish')
    self.skippedState = False
    self.lower_time = 0

  def run(self):
        
    if self.getTime() < 2.0:
      walk_request.noWalk()
      kick_request.setNoKick()
      commands.setStiffness(cfgstiff.One, 0.3)
      return

    st = self.state

    if st.inState(st.stop):
      st.transition(st.checkarms)

    if st.inState(st.checkarms):
      shoulderCutoff = core.DEG_T_RAD * -90
      lpitch = core.joint_values[core.LShoulderPitch] 
      rpitch = core.joint_values[core.RShoulderPitch]
      if lpitch > shoulderCutoff and rpitch > shoulderCutoff:
        st.transition(st.sit)
      else:
        st.transition(st.movearms)
    elif st.inState(st.movearms):
      pose = util.deepcopy(cfgpose.sittingPoseV3)
      for joint, val in cfgpose.armSidePose.items():
        pose[joint] = val
      st.transition(st.sit)
      return ToPoseMoveHead(tilt = 0.0, pose = pose)
    elif st.inState(st.sit):
      self.skippedState = False
      st.transition(st.relaxknee)
      return ToPoseMoveHead(pose = cfgpose.sittingPoseV3)
    elif st.inState(st.relaxknee):
      self.lower_time = self.getTime()
      commands.setStiffness(cfgstiff.ZeroKneeAnklePitch, 0.3)
      st.transition(st.relaxbody)
    elif st.inState(st.relaxbody) and st.timeSinceTransition() > 0.7:
      commands.setStiffness(cfgstiff.Zero, 0.3)
      st.transition(st.finish)
    elif st.inState(st.finish):
      self.finish()

class Stand(Task):
  def reset(self):
    super(Stand, self).reset()
    self.state = state_machine.SimpleStateMachine('requested', 'running')

  def run(self):
    finished = walk_response.finished_standing_
    received = walk_response.received_
    kick_request.setNoKick()
    walk_request.stand()
    if received and self.state.inState('requested'):
      self.state.transition('running')
    elif self.state.inState('running') and finished:
      self.finish()

    if self.getTime() > 2:
      self.finish()

class StandStraight(Task):
  def reset(self):
    super(StandStraight, self).reset()
    self.state = state_machine.SimpleStateMachine('requested', 'running')

  def run(self):
    finished = walk_response.finished_standing_
    received = walk_response.received_
    kick_request.setNoKick()
    walk_request.standStraight()
    if received and self.state.inState('requested'):
      self.state.transition('running')
    elif self.state.inState('running') and finished:
      self.finish()

    if self.getTime() > 2:
      self.finish()

class Squat(Task):
  def __init__(self, time = 3.0):
    super(Squat, self).__init__(time=time)
    self.setChain([ 
      PoseSequence(
        cfgpose.goalieSquatPart1, 1.2,
        cfgpose.goalieSquatPart2, 0.6,
        cfgpose.goalieSquatPart2, time,
        cfgpose.goalieSquat5, 0.6,
        cfgpose.goalieSquat5, 0.9,
        cfgpose.goalieSquatPart2, 0.9,
        cfgpose.goalieSquatGetup15, 1.2,
        cfgpose.goalieSquatGetup2, 1.8,
        cfgpose.goalieSquatGetup7, 0.9
      ),
      Stand()
    ])

class BlockRight(Task):
  def __init__(self, time = 3.0):
    super(BlockRight, self).__init__(time=time)
    self.setSubtask(PoseSequence(
      cfgpose.blockright, 1.0,
      cfgpose.blockright, self.time, 
      cfgpose.sittingPoseNoArms, 2.0,
      cfgpose.standingPose, 2.0
    ))

class BlockLeft(Task):
  def __init__(self, time = 3.0):
    super(BlockLeft, self).__init__(time=time)
    self.setSubtask(PoseSequence(
      cfgpose.blockleft, 1.0,
      cfgpose.blockleft, self.time, 
      cfgpose.sittingPoseNoArms, 2.0,
      cfgpose.standingPose, 2.0
    ))

# class BlockRightAndStand(Task):
#   def __init__(self, time = 3.0):
#     super(BlockRightAndStand, self).__init__(time=time)
#     self.setChain([
#       PoseSequence(
#       cfgpose.blockright, 1.0,
#       cfgpose.blockright, self.time, 
#       #cfgpose.sittingPoseNoArms, 2.0,
#       #cfgpose.standingPose, 2.0
#     ),
#     Sit(),
#     Stand()
#   ])

# class BlockLeftAndStand(Task):
#   def __init__(self, time = 3.0):
#     super(BlockLeftAndStand, self).__init__(time=time)
#     self.setChain([
#       PoseSequence(
#       cfgpose.blockleft, 1.0,
#       cfgpose.blockleft, self.time, 
#       #cfgpose.sittingPoseNoArms, 2.0,
#       #cfgpose.standingPose, 2.0
#     ),
#     Sit(),
#     Stand()
#   ])

class ToPoseArms(MultiTask):
  def __init__(self, pose, time = 0.5):
    self.bpose = ToPose(pose = pose, time = time)
    super(ToPoseArms, self).__init__(self.bpose)

class BlockLeftStand(Task):
  def __init__(self, time = 1.0):
    super(BlockLeftStand, self).__init__(time=time)

    self.state = state_machine.SimpleStateMachine('init', 'put_down', 'finish')

  def run(self):
    commands.setStiffness(cfgstiff.One, 0.3)
    st = self.state

    if st.inState(st.init):
      st.transition(st.put_down)
      return ToPoseArms(cfgpose.raiseLeftArmSide, 0.5)
    elif st.inState(st.put_down):
      st.transition(st.finish)
      return ToPoseArms(cfgpose.bothArmsDown, 0.5)
    elif st.inState(st.finish):
      st.transition(st.init)
      self.finish()
      


class BlockRightStand(Task):
  def __init__(self, time = 1.0):
    super(BlockRightStand, self).__init__(time=time)

    self.state = state_machine.SimpleStateMachine('init', 'put_down', 'finish')

  def run(self):
    commands.setStiffness(cfgstiff.One, 0.3)
    st = self.state

    if st.inState(st.init):
      st.transition(st.put_down)
      return ToPoseArms(cfgpose.raiseRightArmSide, 0.5)
    elif st.inState(st.put_down):
      st.transition(st.finish)
      return ToPoseArms(cfgpose.bothArmsDown, 0.5)
    elif st.inState(st.finish):
      st.transition(st.init)
      self.finish()
      

class BlockCenterStand(Task):
  def __init__(self, time = 1.0):
    super(BlockCenterStand, self).__init__(time=time)

    self.state = state_machine.SimpleStateMachine('init', 'put_down', 'finish')

  def run(self):
    commands.setStiffness(cfgstiff.One, 0.3)
    st = self.state

    if st.inState(st.init):
      st.transition(st.put_down)
      return ToPoseArms(cfgpose.raiseArmsForward, 0.5)
    elif st.inState(st.put_down):
      st.transition(st.finish)
      return ToPoseArms(cfgpose.bothArmsDown, 0.5)
    elif st.inState(st.finish):
      st.transition(st.init)
      self.finish()
      
class BlockRightAndStand(Task):
  def __init__(self, time = 6.0):
    super(BlockRightAndStand, self).__init__(time=time)
    self.state = state_machine.SimpleStateMachine('init', 'blockright', 'sitting', 'standing', 'finish')

  def run(self):
    commands.setStiffness(cfgstiff.One, 0.3)
    st = self.state

    if st.inState(st.init):
      st.transition(st.blockright)
      return ToPose(cfgpose.blockright, 0.5)
    if st.inState(st.blockright):
      st.transition(st.sitting)
      return ToPose(cfgpose.blockright, 0.5)
    elif st.inState(st.sitting):
      st.transition(st.standing)
      return ToPose(cfgpose.sittingPoseNoArms, 3.0)
    elif st.inState(st.standing):
      st.transition(st.finish)
      return ToPose(cfgpose.standingPose, 2.0)
    elif st.inState(st.finish):
      st.transition(st.init)
      self.finish()

class BlockLeftAndStand(Task):
  def __init__(self, time = 6.0):
    super(BlockLeftAndStand, self).__init__(time=time)
    self.state = state_machine.SimpleStateMachine('init', 'blockleft', 'sitting', 'standing', 'finish')

  def run(self):
    commands.setStiffness(cfgstiff.One, 0.3)
    st = self.state

    if st.inState(st.init):
      st.transition(st.blockleft)
      return ToPose(cfgpose.blockleft, 0.5)
    elif st.inState(st.blockleft):
      st.transition(st.sitting)
      return ToPose(cfgpose.blockleft, 0.5)
    elif st.inState(st.sitting):
      st.transition(st.standing)
      return ToPose(cfgpose.sittingPoseNoArms_v2, 3.0)
    elif st.inState(st.standing):
      st.transition(st.finish)
      return ToPose(cfgpose.standingPose, 2.0)
    elif st.inState(st.finish):
      st.transition(st.init)
      self.finish()