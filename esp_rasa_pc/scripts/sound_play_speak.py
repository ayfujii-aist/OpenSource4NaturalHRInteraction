#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from std_srvs.srv import Empty
import time

class SoundPlaySpeak(object):
    def __init__(self):
        self.client = SoundClient()
        self.pub = rospy.Publisher('/audio_recognition_status', String, queue_size=10)
        rospy.Subscriber("/rasa_ros/agent_speech", String, self.topic_cb)

    def topic_cb(self, msg):
        rospy.loginfo("start speaking")
        self.client.say(msg.data)
        sleep_time = len(msg.data) / 8 + 1
        time.sleep(sleep_time)
        rospy.loginfo("finish speaking")
        self.pub.publish("Restart")

if __name__ == '__main__':
    rospy.init_node("sound_client_speak")
    SoundPlaySpeak()
    rospy.spin()
