#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from audio_common_msgs.msg import AudioData
import array
from std_msgs.msg import String
import time

class AudioDetect(object):
    def __init__(self):
        self.type_code = {}
        for code in ['b', 'h', 'i', 'l']:
            self.type_code[array.array(code).itemsize] = code
        self.depth = rospy.get_param("~depth", 16)
        self.dtype = self.type_code[self.depth/8]
        self.volume_threshold = rospy.get_param("~volume_threshold", 500)
        self.detection = 0
        self.last_detection = time.time()
        self.speak_state=0

        self.pub =  rospy.Publisher("/audio_detection", String, queue_size=10)
        rospy.Subscriber("/audio", AudioData, self.topic_cb)
        rospy.Subscriber("/audio_recognition_status", String, self.topic_cb2)

    def topic_cb(self, msg):
        detection_state = String()
        data = array.array(self.dtype, bytes(msg.data)).tolist()
        if time.time() - self.last_detection > 10 and self.speak_state==1 :
            self.speak_state=0
            print("Restart audio detection by time")
        if self.speak_state==0:
            if max(data) > self.volume_threshold:
                detection_state.data="Start Detection"
                self.detection = 1
                self.last_detection = time.time()
            else:
                if self.detection==0:
                    detection_state.data="None"
                elif self.detection==1:
                    if time.time()-self.last_detection > 0.8:
                        detection_state.data="Detect"
                        self.last_detection = time.time()
                        self.detection=0
                        self.speak_state=1
                        print("audio detected")
                    else:
                        detection_state.data="Continue Detection"
            self.pub.publish(detection_state)

    def topic_cb2(self, msg):
        if msg.data=="Restart":
            self.speak_state=0
            print("restart audio detection")

if __name__ == '__main__':
    rospy.init_node('audio_detect')
    AudioDetect()
    rospy.spin()
