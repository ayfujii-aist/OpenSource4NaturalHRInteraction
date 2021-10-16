#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from naoqi_bridge_msgs.msg import AudioBuffer
from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import Image

class SubscribeSensors(object):
    def __init__(self):
        rospy.Subscriber("/nao_robot/naoqi_driver/audio_raw", AudioData, self.topic_cb)
        rospy.Subscriber("/nao_robot/naoqi_driver/audio", AudioBuffer, self.topic_cb)
        rospy.Subscriber("/nao_robot/naoqi_driver/camera/bottom/image_raw", Image, self.topic_cb)
        rospy.Subscriber("/nao_robot/naoqi_driver/camera/front/image_raw", Image, self.topic_cb)

    def topic_cb(self, msg):
        return None

if __name__ == '__main__':
    rospy.init_node('subscribe_sensors')
    SubscribeSensors()
    rospy.spin()
