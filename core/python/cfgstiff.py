#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core

Head = [0] * core.NUM_JOINTS
Head[core.HeadYaw] = 1
Head[core.HeadPitch] = 1

ALWalk = [0] * core.NUM_JOINTS
ALWalk[core.HeadYaw] = 1
ALWalk[core.HeadPitch] = 1
ALWalk[core.LShoulderPitch] = 0.7
ALWalk[core.LShoulderRoll] = 0.7
ALWalk[core.LElbowYaw] = 0.7
ALWalk[core.LElbowRoll] = 0.7
ALWalk[core.LHipYawPitch] = 1.0
ALWalk[core.LHipPitch] = 1.0
ALWalk[core.LHipRoll] = 1.0
ALWalk[core.LKneePitch] = 0.8
ALWalk[core.LAnklePitch] = 0.8
ALWalk[core.LAnkleRoll] = 0.8
ALWalk[core.RHipYawPitch] = 1.0
ALWalk[core.RHipPitch] = 1.0
ALWalk[core.RHipRoll] = 1.0
ALWalk[core.RKneePitch] = 0.8
ALWalk[core.RAnklePitch] = 0.8
ALWalk[core.RAnkleRoll] = 0.8
ALWalk[core.RShoulderPitch] = 0.7
ALWalk[core.RShoulderRoll] = 0.7
ALWalk[core.RElbowYaw] = 0.7
ALWalk[core.RElbowRoll] = 0.7


One = [0] * core.NUM_JOINTS
One[core.HeadYaw] = 1
One[core.HeadPitch] = 1
One[core.LShoulderPitch] = 1
One[core.LShoulderRoll] = 1
One[core.LElbowYaw] = 1
One[core.LElbowRoll] = 1
One[core.LHipYawPitch] = 1
One[core.LHipPitch] = 1
One[core.LHipRoll] = 1
One[core.LKneePitch] = 1
One[core.LAnklePitch] = 1
One[core.LAnkleRoll] = 1
One[core.RHipYawPitch] = 1
One[core.RHipPitch] = 1
One[core.RHipRoll] = 1
One[core.RKneePitch] = 1
One[core.RAnklePitch] = 1
One[core.RAnkleRoll] = 1
One[core.RShoulderPitch] = 1
One[core.RShoulderRoll] = 1
One[core.RElbowYaw] = 1
One[core.RElbowRoll] = 1


Zero = [0] * core.NUM_JOINTS
Zero[core.HeadYaw] = 0
Zero[core.HeadPitch] = 0
Zero[core.LShoulderPitch] = 0
Zero[core.LShoulderRoll] = 0
Zero[core.LElbowYaw] = 0
Zero[core.LElbowRoll] = 0
Zero[core.LHipYawPitch] = 0
Zero[core.LHipPitch] = 0
Zero[core.LHipRoll] = 0
Zero[core.LKneePitch] = 0
Zero[core.LAnklePitch] = 0
Zero[core.LAnkleRoll] = 0
Zero[core.RHipYawPitch] = 0
Zero[core.RHipPitch] = 0
Zero[core.RHipRoll] = 0
Zero[core.RKneePitch] = 0
Zero[core.RAnklePitch] = 0
Zero[core.RAnkleRoll] = 0
Zero[core.RShoulderPitch] = 0
Zero[core.RShoulderRoll] = 0
Zero[core.RElbowYaw] = 0
Zero[core.RElbowRoll] = 0

ZeroPtOne = [0] * core.NUM_JOINTS
ZeroPtOne[core.HeadYaw] = 0.1
ZeroPtOne[core.HeadPitch] = 0.1
ZeroPtOne[core.LShoulderPitch] = 0.1
ZeroPtOne[core.LShoulderRoll] = 0.1
ZeroPtOne[core.LElbowYaw] = 0.1
ZeroPtOne[core.LElbowRoll] = 0.1
ZeroPtOne[core.LHipYawPitch] = 0.1
ZeroPtOne[core.LHipPitch] = 0.1
ZeroPtOne[core.LHipRoll] = 0.1
ZeroPtOne[core.LKneePitch] = 0.1
ZeroPtOne[core.LAnklePitch] = 0.1
ZeroPtOne[core.LAnkleRoll] = 0.1
ZeroPtOne[core.RHipYawPitch] = 0.1
ZeroPtOne[core.RHipPitch] = 0.1
ZeroPtOne[core.RHipRoll] = 0.1
ZeroPtOne[core.RKneePitch] = 0.1
ZeroPtOne[core.RAnklePitch] = 0.1
ZeroPtOne[core.RAnkleRoll] = 0.1
ZeroPtOne[core.RShoulderPitch] = 0.1
ZeroPtOne[core.RShoulderRoll] = 0.1
ZeroPtOne[core.RElbowYaw] = 0.1
ZeroPtOne[core.RElbowRoll] = 0.1

ZeroPtThree = [0] * core.NUM_JOINTS
ZeroPtThree[core.HeadYaw] = 0.3
ZeroPtThree[core.HeadPitch] = 0.3
ZeroPtThree[core.LShoulderPitch] = 0.3
ZeroPtThree[core.LShoulderRoll] = 0.3
ZeroPtThree[core.LElbowYaw] = 0.3
ZeroPtThree[core.LElbowRoll] = 0.3
ZeroPtThree[core.LHipYawPitch] = 0.3
ZeroPtThree[core.LHipPitch] = 0.3
ZeroPtThree[core.LHipRoll] = 0.3
ZeroPtThree[core.LKneePitch] = 0.3
ZeroPtThree[core.LAnklePitch] = 0.3
ZeroPtThree[core.LAnkleRoll] = 0.3
ZeroPtThree[core.RHipYawPitch] = 0.3
ZeroPtThree[core.RHipPitch] = 0.3
ZeroPtThree[core.RHipRoll] = 0.3
ZeroPtThree[core.RKneePitch] = 0.3
ZeroPtThree[core.RAnklePitch] = 0.3
ZeroPtThree[core.RAnkleRoll] = 0.3
ZeroPtThree[core.RShoulderPitch] = 0.3
ZeroPtThree[core.RShoulderRoll] = 0.3
ZeroPtThree[core.RElbowYaw] = 0.3
ZeroPtThree[core.RElbowRoll] = 0.3

ZeroKneeAnklePitch = [0] * core.NUM_JOINTS
ZeroKneeAnklePitch[core.HeadYaw] = 1.0
ZeroKneeAnklePitch[core.HeadPitch] = 1.0
ZeroKneeAnklePitch[core.LShoulderPitch] = 1.0
ZeroKneeAnklePitch[core.LShoulderRoll] = 1.0
ZeroKneeAnklePitch[core.LElbowYaw] = 1.0
ZeroKneeAnklePitch[core.LElbowRoll] = 1.0
ZeroKneeAnklePitch[core.LHipYawPitch] = 1.0
ZeroKneeAnklePitch[core.LHipPitch] = 1.0
ZeroKneeAnklePitch[core.LHipRoll] = 1.0
ZeroKneeAnklePitch[core.LKneePitch] = 0
ZeroKneeAnklePitch[core.LAnklePitch] = 0
ZeroKneeAnklePitch[core.LAnkleRoll] = 1.0
ZeroKneeAnklePitch[core.RHipYawPitch] = 1.0
ZeroKneeAnklePitch[core.RHipPitch] = 1.0
ZeroKneeAnklePitch[core.RHipRoll] = 1.0
ZeroKneeAnklePitch[core.RKneePitch] = 0
ZeroKneeAnklePitch[core.RAnklePitch] = 0
ZeroKneeAnklePitch[core.RAnkleRoll] = 1.0
ZeroKneeAnklePitch[core.RShoulderPitch] = 1.0
ZeroKneeAnklePitch[core.RShoulderRoll] = 1.0
ZeroKneeAnklePitch[core.RElbowYaw] = 1.0
ZeroKneeAnklePitch[core.RElbowRoll] = 1.0

LowArms = [0] * core.NUM_JOINTS
LowArms[core.HeadYaw] = 1
LowArms[core.HeadPitch] = 1
LowArms[core.LShoulderPitch] = 0.1
LowArms[core.LShoulderRoll] = 0.1
LowArms[core.LElbowYaw] = 0.1
LowArms[core.LElbowRoll] = 0.1
LowArms[core.LHipYawPitch] = 1
LowArms[core.LHipPitch] = 1
LowArms[core.LHipRoll] = 1
LowArms[core.LKneePitch] = 1
LowArms[core.LAnklePitch] = 1
LowArms[core.LAnkleRoll] = 1
LowArms[core.RHipYawPitch] = 1
LowArms[core.RHipPitch] = 1
LowArms[core.RHipRoll] = 1
LowArms[core.RKneePitch] = 1
LowArms[core.RAnklePitch] = 1
LowArms[core.RAnkleRoll] = 1
LowArms[core.RShoulderPitch] = 0.1
LowArms[core.RShoulderRoll] = 0.1
LowArms[core.RElbowYaw] = 0.1
LowArms[core.RElbowRoll] = 0.1


Stand = [0] * core.NUM_JOINTS
Stand[core.HeadYaw] = 1
Stand[core.HeadPitch] = 1
Stand[core.LShoulderPitch] = 0.6
Stand[core.LShoulderRoll] = 0.6
Stand[core.LElbowYaw] = 0.6
Stand[core.LElbowRoll] = 0.6
Stand[core.LHipYawPitch] = 0.6
Stand[core.LHipPitch] = 0.6
Stand[core.LHipRoll] = 0.6
Stand[core.LKneePitch] = 0.6
Stand[core.LAnklePitch] = 0.6
Stand[core.LAnkleRoll] = 0.6
Stand[core.RHipYawPitch] = 0.6
Stand[core.RHipPitch] = 0.6
Stand[core.RHipRoll] = 0.6
Stand[core.RKneePitch] = 0.6
Stand[core.RAnklePitch] = 0.6
Stand[core.RAnkleRoll] = 0.6
Stand[core.RShoulderPitch] = 0.6
Stand[core.RShoulderRoll] = 0.6
Stand[core.RElbowYaw] = 0.6
Stand[core.RElbowRoll] = 0.6


KeeperStand = [0] * core.NUM_JOINTS
KeeperStand[core.HeadYaw] = 1
KeeperStand[core.HeadPitch] = 1
KeeperStand[core.LShoulderPitch] = 0.6
KeeperStand[core.LShoulderRoll] = 0.6
KeeperStand[core.LElbowYaw] = 0.6
KeeperStand[core.LElbowRoll] = 0.6
KeeperStand[core.LHipYawPitch] = 0.6
KeeperStand[core.LHipPitch] = 0.6
KeeperStand[core.LHipRoll] = 0.6
KeeperStand[core.LKneePitch] = 0.6
KeeperStand[core.LAnklePitch] = 0.6
KeeperStand[core.LAnkleRoll] = 0.6
KeeperStand[core.RHipYawPitch] = 0.6
KeeperStand[core.RHipPitch] = 0.6
KeeperStand[core.RHipRoll] = 0.6
KeeperStand[core.RKneePitch] = 0.6
KeeperStand[core.RAnklePitch] = 0.6
KeeperStand[core.RAnkleRoll] = 0.6
KeeperStand[core.RShoulderPitch] = 0.6
KeeperStand[core.RShoulderRoll] = 0.6
KeeperStand[core.RElbowYaw] = 0.6
KeeperStand[core.RElbowRoll] = 0.6

OldKick = [0] * core.NUM_JOINTS
OldKick[core.HeadYaw] = 1
OldKick[core.HeadPitch] = 1
OldKick[core.LShoulderPitch] = 0.25
OldKick[core.LShoulderRoll] = 0.25
OldKick[core.LElbowYaw] = 0.25
OldKick[core.LElbowRoll] = 0.25
OldKick[core.LHipYawPitch] = 1
OldKick[core.LHipPitch] = 0.75
OldKick[core.LHipRoll] = 0.75
OldKick[core.LKneePitch] = 0.75
OldKick[core.LAnklePitch] = 0.75
OldKick[core.LAnkleRoll] = 0.75
OldKick[core.RHipYawPitch] = 1
OldKick[core.RHipPitch] = 0.75
OldKick[core.RHipRoll] = 0.75
OldKick[core.RKneePitch] = 0.75
OldKick[core.RAnklePitch] = 0.75
OldKick[core.RAnkleRoll] = 0.75
OldKick[core.RShoulderPitch] = 0.25
OldKick[core.RShoulderRoll] = 0.25
OldKick[core.RElbowYaw] = 0.25
OldKick[core.RElbowRoll] = 0.25

OldKickLegOne = [0] * core.NUM_JOINTS
OldKickLegOne[core.HeadYaw] = 1
OldKickLegOne[core.HeadPitch] = 1
OldKickLegOne[core.LShoulderPitch] = 0.25
OldKickLegOne[core.LShoulderRoll] = 0.25
OldKickLegOne[core.LElbowYaw] = 0.25
OldKickLegOne[core.LElbowRoll] = 0.25
OldKickLegOne[core.LHipYawPitch] = 1
OldKickLegOne[core.LHipPitch] = 1.0
OldKickLegOne[core.LHipRoll] = 1.0
OldKickLegOne[core.LKneePitch] = 1.0
OldKickLegOne[core.LAnklePitch] = 1.0
OldKickLegOne[core.LAnkleRoll] = 1.0
OldKickLegOne[core.RHipYawPitch] = 1
OldKickLegOne[core.RHipPitch] = 1.0
OldKickLegOne[core.RHipRoll] = 1.0
OldKickLegOne[core.RKneePitch] = 1.0
OldKickLegOne[core.RAnklePitch] = 1.0
OldKickLegOne[core.RAnkleRoll] = 1.0
OldKickLegOne[core.RShoulderPitch] = 0.25
OldKickLegOne[core.RShoulderRoll] = 0.25
OldKickLegOne[core.RElbowYaw] = 0.25
OldKickLegOne[core.RElbowRoll] = 0.25

OldKickRightLegOne = [0] * core.NUM_JOINTS
OldKickRightLegOne[core.HeadYaw] = 1
OldKickRightLegOne[core.HeadPitch] = 1
OldKickRightLegOne[core.LShoulderPitch] = 0.25
OldKickRightLegOne[core.LShoulderRoll] = 0.25
OldKickRightLegOne[core.LElbowYaw] = 0.25
OldKickRightLegOne[core.LElbowRoll] = 0.25
OldKickRightLegOne[core.LHipYawPitch] = 1
OldKickRightLegOne[core.LHipPitch] = 0.75
OldKickRightLegOne[core.LHipRoll] = 0.75
OldKickRightLegOne[core.LKneePitch] = 0.75
OldKickRightLegOne[core.LAnklePitch] = 0.75
OldKickRightLegOne[core.LAnkleRoll] = 0.75
OldKickRightLegOne[core.RHipYawPitch] = 1
OldKickRightLegOne[core.RHipPitch] = 1.0
OldKickRightLegOne[core.RHipRoll] = 1.0
OldKickRightLegOne[core.RKneePitch] = 1.0
OldKickRightLegOne[core.RAnklePitch] = 1.0
OldKickRightLegOne[core.RAnkleRoll] = 1.0
OldKickRightLegOne[core.RShoulderPitch] = 0.25
OldKickRightLegOne[core.RShoulderRoll] = 0.25
OldKickRightLegOne[core.RElbowYaw] = 0.25
OldKickRightLegOne[core.RElbowRoll] = 0.25

OldKickLeftLegOne = [0] * core.NUM_JOINTS
OldKickLeftLegOne[core.HeadYaw] = 1
OldKickLeftLegOne[core.HeadPitch] = 1
OldKickLeftLegOne[core.LShoulderPitch] = 0.25
OldKickLeftLegOne[core.LShoulderRoll] = 0.25
OldKickLeftLegOne[core.LElbowYaw] = 0.25
OldKickLeftLegOne[core.LElbowRoll] = 0.25
OldKickLeftLegOne[core.LHipYawPitch] = 1
OldKickLeftLegOne[core.LHipPitch] = 1.0
OldKickLeftLegOne[core.LHipRoll] = 1.0
OldKickLeftLegOne[core.LKneePitch] = 1.0
OldKickLeftLegOne[core.LAnklePitch] = 1.0
OldKickLeftLegOne[core.LAnkleRoll] = 1.0
OldKickLeftLegOne[core.RHipYawPitch] = 1
OldKickLeftLegOne[core.RHipPitch] = 0.75
OldKickLeftLegOne[core.RHipRoll] = 0.75
OldKickLeftLegOne[core.RKneePitch] = 0.75
OldKickLeftLegOne[core.RAnklePitch] = 0.75
OldKickLeftLegOne[core.RAnkleRoll] = 0.75
OldKickLeftLegOne[core.RShoulderPitch] = 0.25
OldKickLeftLegOne[core.RShoulderRoll] = 0.25
OldKickLeftLegOne[core.RElbowYaw] = 0.25
OldKickLeftLegOne[core.RElbowRoll] = 0.25

StandHeadFree = [0] * core.NUM_JOINTS
StandHeadFree[core.HeadYaw] = 0
StandHeadFree[core.HeadPitch] = 0
StandHeadFree[core.LShoulderPitch] = 0.1
StandHeadFree[core.LShoulderRoll] = 0.1
StandHeadFree[core.LElbowYaw] = 0.1
StandHeadFree[core.LElbowRoll] = 0.1
StandHeadFree[core.LHipYawPitch] = 0.6
StandHeadFree[core.LHipPitch] = 0.6
StandHeadFree[core.LHipRoll] = 0.6
StandHeadFree[core.LKneePitch] = 0.6
StandHeadFree[core.LAnklePitch] = 0.6
StandHeadFree[core.LAnkleRoll] = 0.6
StandHeadFree[core.RHipYawPitch] = 0.6
StandHeadFree[core.RHipPitch] = 0.6
StandHeadFree[core.RHipRoll] = 0.6
StandHeadFree[core.RKneePitch] = 0.6
StandHeadFree[core.RAnklePitch] = 0.6
StandHeadFree[core.RAnkleRoll] = 0.6
StandHeadFree[core.RShoulderPitch] = 0.1
StandHeadFree[core.RShoulderRoll] = 0.1
StandHeadFree[core.RElbowYaw] = 0.1
StandHeadFree[core.RElbowRoll] = 0.1

LegsOffRestWalk = [0] * core.NUM_JOINTS
LegsOffRestWalk[core.HeadYaw] = 1
LegsOffRestWalk[core.HeadPitch] = 1
LegsOffRestWalk[core.LHipYawPitch] = 0
LegsOffRestWalk[core.LHipPitch] = 0
LegsOffRestWalk[core.LHipRoll] = 0
LegsOffRestWalk[core.LKneePitch] = 0
LegsOffRestWalk[core.LAnklePitch] = 0
LegsOffRestWalk[core.LAnkleRoll] = 0
LegsOffRestWalk[core.RHipYawPitch] = 0
LegsOffRestWalk[core.RHipPitch] = 0
LegsOffRestWalk[core.RHipRoll] = 0
LegsOffRestWalk[core.RKneePitch] = 0
LegsOffRestWalk[core.RAnklePitch] = 0
LegsOffRestWalk[core.RAnkleRoll] = 0
LegsOffRestWalk[core.LShoulderPitch] = 0.7
LegsOffRestWalk[core.LShoulderRoll] = 0.7
LegsOffRestWalk[core.LElbowYaw] = 0.7
LegsOffRestWalk[core.LElbowRoll] = 0.7
LegsOffRestWalk[core.RShoulderPitch] = 0.7
LegsOffRestWalk[core.RShoulderRoll] = 0.7
LegsOffRestWalk[core.RElbowYaw] = 0.7
LegsOffRestWalk[core.RElbowRoll] = 0.7

RightLegOffRestWalk = [0] * core.NUM_JOINTS
RightLegOffRestWalk[core.HeadYaw] = 1
RightLegOffRestWalk[core.HeadPitch] = 1
RightLegOffRestWalk[core.LHipYawPitch] = 1.0
RightLegOffRestWalk[core.LHipPitch] = 1.0
RightLegOffRestWalk[core.LHipRoll] = 1.0
RightLegOffRestWalk[core.LKneePitch] = 0.8
RightLegOffRestWalk[core.LAnklePitch] = 0.8
RightLegOffRestWalk[core.LAnkleRoll] = 0.8
RightLegOffRestWalk[core.RHipYawPitch] = 0
RightLegOffRestWalk[core.RHipPitch] = 0
RightLegOffRestWalk[core.RHipRoll] = 0
RightLegOffRestWalk[core.RKneePitch] = 0
RightLegOffRestWalk[core.RAnklePitch] = 0
RightLegOffRestWalk[core.RAnkleRoll] = 0
RightLegOffRestWalk[core.LShoulderPitch] = 0.7
RightLegOffRestWalk[core.LShoulderRoll] = 0.7
RightLegOffRestWalk[core.LElbowYaw] = 0.7
RightLegOffRestWalk[core.LElbowRoll] = 0.7
RightLegOffRestWalk[core.RShoulderPitch] = 0.7
RightLegOffRestWalk[core.RShoulderRoll] = 0.7
RightLegOffRestWalk[core.RElbowYaw] = 0.7
RightLegOffRestWalk[core.RElbowRoll] = 0.7
