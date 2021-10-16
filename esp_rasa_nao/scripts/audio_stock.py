#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from audio_common_msgs.msg import AudioData
import array
from std_msgs.msg import String
import time

class AudioStock(object):
    def __init__(self):
        self.audio_data = []
        self.type_code = {}
        for code in ['b', 'h', 'i', 'l']:
            self.type_code[array.array(code).itemsize] = code
        self.depth = rospy.get_param("~depth", 16)
        self.n_channel = rospy.get_param("~channels", 1)
        self.sample_rate = rospy.get_param("~sample_rate", 48000)
        self.dtype = self.type_code[self.depth/8]

        rospy.Subscriber("/audio_recognize", AudioData, self.topic_cb)
        rospy.Subscriber("/speech_status", String, self.zero_cb)
        self.pub =  rospy.Publisher("/audio_for_espnet", AudioData, queue_size=1)

    def topic_cb(self, msg):
        data = array.array(self.dtype, bytes(msg.data)).tolist()
        chan_data = data[0::self.n_channel]
        str_data = array.array(self.dtype, chan_data).tostring()
        int_list = [ord(i) for i in str_data]
        self.audio_data.extend(int_list)

        # 5sec x hz
        if len(self.audio_data) > 5 * self.sample_rate:
            self.audio_data = self.audio_data[(len(self.audio_data) - 5*self.sample_rate):]
        audio_esp = AudioData()
        audio_esp.data = self.audio_data
        self.pub.publish(audio_esp)

    def zero_cb(self, msg):
        if msg.data=="done":
            self.audio_data=[]
            print("clear audio data stock")

if __name__ == '__main__':
    rospy.init_node('audio_stock')
    AudioStock()
    rospy.spin()
