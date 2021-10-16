#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from rasa_ros.msg import RasaMsg
from std_msgs.msg import String
from std_srvs.srv import Empty
import time

class NaoSpeakRasa(object):
    def __init__(self):
        self.nao_speech_pub = rospy.Publisher('/speech', String, queue_size=10)
        rospy.Subscriber("/rasa_ros/agent_speech", String, self.topic_cb)
        rospy.wait_for_service("/passthrough_node/request")
        rospy.wait_for_service("/passthrough_node/stop")
        rospy.wait_for_service("/passthrough_zero/request")
        rospy.wait_for_service("/passthrough_zero/stop")
        self.start_audio = rospy.ServiceProxy("/passthrough_node/request", Empty)
        self.stop_audio = rospy.ServiceProxy("/passthrough_node/stop", Empty)
        self.start_zero = rospy.ServiceProxy("/passthrough_zero/request", Empty)
        self.stop_zero = rospy.ServiceProxy("/passthrough_zero/stop", Empty)
        self.start_audio()

    def topic_cb(self, msg):
        self.speak_state = 1
        self.stop_audio()
        self.start_zero()
        nao_speech = String()
        nao_speech.data = msg.data
        self.nao_speech_pub.publish(nao_speech)
        self.sub_status=rospy.Subscriber("/speech_status", String, self.cb_once)
        while self.speak_state ==1:
            time.sleep(0.1)
        rospy.loginfo("finish speaking")
        self.stop_zero()
        self.start_audio()

    def cb_once(self, msg):
        if msg.data=="done":
            self.speak_state=0
            self.sub_status.unregister()

if __name__ == '__main__':
    rospy.init_node('nao_speak_rasa')
    NaoSpeakRasa()
    rospy.spin()

