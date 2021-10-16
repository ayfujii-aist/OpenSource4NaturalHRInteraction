#!/usr/bin/env python3
# coding: utf-8
#from __future__ import unicode_literals

"""Custom actions"""
from rasa_sdk.interfaces import Action
from typing import Dict, Text, Any, List
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rasa_sdk import Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet
from rasa_sdk.events import ActiveLoop
from rasa_sdk.events import AllSlotsReset
from rasa_sdk.events import LoopInterrupted
import rospy
import json
import random
import os
import time
import rospkg
import sys

all_joints=["HeadYaw", "HeadPitch",
            "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
            "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
            "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
            "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
reset_pose_angles = [0, -0.15,
                     1.4, 0.2, -1.2, -0.4, 0.1, 0.3,
                     -0.2, 0.1, 0.1, -0.1, 0.1, -0.1,
                     -0.2, -0.1, 0.1, -0.1, 0.1, 0.1,
                     1.4, -0.2, 1.2, 0.4, 0.1, 0.3]

upper_joints=["HeadYaw", "HeadPitch",
              "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
              "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
reset_pose_upper_angles = [0, -0.15,
                           1.4, 0.2, -1.2, -0.4, 0.1, 0.3,
                           1.4, -0.2, 1.2, 0.4, 0.1, 0.3]

class ActionResetPose(Action):
    def __init__(self):
        self.pub_nao=rospy.Publisher("/nao_robot/pose/joint_angles" ,JointAnglesWithSpeed, queue_size=0)
        try:
            rospy.init_node("nao_rasa_action", anonymous=True)
        except:
            pass

    def name(self) -> Text:
        """Unique identifier of the action"""
        return "gesture_reset_pose"
    async def run(
            self,
            dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]
    ) -> List[Dict]:
        print("[rasa_action] reset pose")
        j=JointAnglesWithSpeed()
        j.joint_names  = all_joints
        j.joint_angles  = reset_pose_angles
        j.speed  = 0.1
        j.relative = 0
        self.pub_nao.publish(j)
        return

class ActionResetUpperPose(Action):
    def __init__(self):
        self.pub_nao=rospy.Publisher("/nao_robot/pose/joint_angles" ,JointAnglesWithSpeed, queue_size=0)
        try:
            rospy.init_node("nao_rasa_action", anonymous=True)
        except:
            pass

    def name(self) -> Text:
        """Unique identifier of the action"""
        return "gesture_reset_upper_pose"
    async def run(
            self,
            dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]
    ) -> List[Dict]:
        print("[rasa_action] reset upper pose")
        j=JointAnglesWithSpeed()
        j.joint_names  = upper_joints
        j.joint_angles  = reset_pose_upper_angles
        j.speed  = 0.1
        j.relative = 0
        self.pub_nao.publish(j)
        return

class ActionHeadMoveLeft(Action):
    def __init__(self):
        self.pub_nao=rospy.Publisher("/nao_robot/pose/joint_angles" ,JointAnglesWithSpeed, queue_size=0)
        try:
            rospy.init_node("nao_rasa_action", anonymous=True)
        except:
            pass

    def name(self) -> Text:
        """Unique identifier of the action"""
        return "gesture_head_move_left"

    async def run(
            self,
            dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]
    ) -> List[Dict]:
        print("[rasa_action] move head left")
        j=JointAnglesWithSpeed()
        j.joint_names  = ["HeadYaw", "HeadPitch"]
        j.joint_angles  = [0.2, -0.2]
        j.speed  = 0.1
        j.relative = 0
        self.pub_nao.publish(j)
        return

class ActionHeadMoveRight(Action):
    def __init__(self):
        self.pub_nao=rospy.Publisher("/nao_robot/pose/joint_angles" ,JointAnglesWithSpeed, queue_size=0)
        try:
            rospy.init_node("nao_rasa_action", anonymous=True)
        except:
            pass

    def name(self) -> Text:
        """Unique identifier of the action"""
        return "gesture_head_move_right"
    async def run(
            self,
            dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]
    ) -> List[Dict]:
        print("[rasa_action] move head right")
        j=JointAnglesWithSpeed()
        j.joint_names  = ["HeadYaw", "HeadPitch"]
        j.joint_angles  = [-0.2, -0.2]
        j.speed  = 0.1
        j.relative = 0
        self.pub_nao.publish(j)
        return

class ActionHeadMoveNodding(Action):
    def __init__(self):
        self.pub_nao=rospy.Publisher("/nao_robot/pose/joint_angles" ,JointAnglesWithSpeed, queue_size=0)
        try:
            rospy.init_node("nao_rasa_action", anonymous=True)
        except:
            pass

    def name(self) -> Text:
        """Unique identifier of the action"""
        return "gesture_head_nodding"
    async def run(
            self,
            dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]
    ) -> List[Dict]:
        print("[rasa_action] move head right")
        j=JointAnglesWithSpeed()
        j.joint_names  = ["HeadYaw", "HeadPitch"]
        j.joint_angles  = [0, 0.2]
        j.speed  = 0.2
        j.relative = 0
        self.pub_nao.publish(j)
        time.sleep(0.5)
        j.joint_angles = [0, -0.2]
        self.pub_nao.publish(j)
        return

class ActionMoveLR(Action):
    def __init__(self):
        self.pub_nao=rospy.Publisher("/nao_robot/pose/joint_angles" ,JointAnglesWithSpeed, queue_size=0)
        try:
            rospy.init_node("nao_rasa_action", anonymous=True)
        except:
            pass

    def name(self) -> Text:
        """Unique identifier of the action"""
        return "gesture_move_LR"

    async def run(
            self,
            dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]
    ) -> List[Dict]:
        self.sub_joint=rospy.Subscriber("/joint_states", JointState, self.topic_cb)
        return

    def topic_cb(self, msg):
        HeadYaw=msg.position[0]
        #print("HeadYawAngle = " + str(HeadYaw))
        j=JointAnglesWithSpeed()
        if HeadYaw>0:
            #look right
            print("[rasa_action] move right")
            j.joint_names  = ["HeadYaw", "HeadPitch",
                              "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
                              "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
            j.joint_angles  = [-0.2, -0.2,
                               0.3, -0.5, 2.0, 1.0,
                               1.5, 0.2, -1.2, -0.4]
            j.speed  = 0.1
            j.relative = 0
        else:
            #look left
            print("[rasa_action] move left")
            j.joint_names  = ["HeadYaw", "HeadPitch",
                              "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
                              "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
            j.joint_angles  = [0.2, -0.2,
                               1.5, -0.2, 1.2, 0.4,
                               0.3, 0.5, -2.0, -1.0]
            j.speed  = 0.1
            j.relative = 0
        self.pub_nao.publish(j)
        self.sub_joint.unregister()

class ActionMoveBotharm(Action):
    def __init__(self):
        self.pub_nao=rospy.Publisher("/nao_robot/pose/joint_angles" ,JointAnglesWithSpeed, queue_size=0)
        try:
            rospy.init_node("nao_rasa_action", anonymous=True)
        except:
            pass

    def name(self) -> Text:
        """Unique identifier of the action"""
        return "gesture_move_botharm"

    async def run(
            self,
            dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]
    ) -> List[Dict]:
        self.sub_joint=rospy.Subscriber("/joint_states", JointState, self.topic_cb)
        return

    def topic_cb(self, msg):
        j=JointAnglesWithSpeed()
        print("[rasa_action] move both hands")
        j.joint_names  = ["HeadYaw", "HeadPitch",
                          "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
                          "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
        j.joint_angles  = [0, -0.2,
                           0.3, -0.5, 2.0, 1.0,
                           0.3, 0.5, -2.0, -1.0]
        j.speed  = 0.1
        j.relative = 0
        self.pub_nao.publish(j)
        self.sub_joint.unregister()

