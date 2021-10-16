#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALBroker, ALProxy, ALModule)
from std_msgs.msg import String

import time
import rospkg
import sys

class AudioDetect(NaoqiNode):
    #This should be treated as a constant
    NODE_NAME = "audio_detect"

    def __init__( self ):
        NaoqiNode.__init__( self, self.NODE_NAME )
        self.proxy = self.get_proxy("ALSoundDetection")
        self.proxy.subscribe("SoundDetected")
        self.proxy.setParameter("Sensitivity", rospy.get_param("~volume_threshold", 0.9))
        self.memory = self.get_proxy("ALMemory")
        self.motion = self.get_proxy("ALMotion")

        state_pub = rospy.Publisher("/audio_detection", String, queue_size=1)
        rospy.Subscriber("/speech_status", String, self.topic_cb1)
        rospy.Subscriber("/audio_recognition_status", String, self.topic_cb2)

        r = rospy.Rate(10)
        p_sound = 0
        self.speak_state=0

        while not rospy.is_shutdown():
            try:
                sound = self.memory.getData("SoundDetected")[-1][1]
                state = String()
                if self.speak_state==0:
                    if sound < p_sound:
                        state.data = "Detect"
                        print("Detect volume")
                        self.speak_state=1
                        time.sleep(0.2)
                        state_pub.publish(state)

                    else:
                        state.data = "None"
                        state_pub.publish(state)
                p_sound = sound
            except:
                pass
            r.sleep()


    def topic_cb1(self, msg):
        if msg.data=="done":
            self.speak_state=0
            print("restart audio detection")

    def topic_cb2(self, msg):
        if msg.data=="Restart":
            self.speak_state=0
            print("restart audio detection")


if __name__ == '__main__':
    node = AudioDetect()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
