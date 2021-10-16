#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from audio_common_msgs.msg import AudioData

def ad0_pub():
    rospy.init_node('ad0_publisher')
    ad0_pub = rospy.Publisher('/audio_zero', AudioData, queue_size=1)
    audio_zero = AudioData()
    r = rospy.Rate(10)
    sample_rate = rospy.get_param("~sample_rate", 48000)
    n_channel = rospy.get_param("~channels", 1)
    while not rospy.is_shutdown():
        zeros = np.zeros(sample_rate * n_channel *2 / 10)
        audio_zero.data = zeros.tolist()
        ad0_pub.publish(audio_zero)
        r.sleep()

if __name__ == '__main__':
    try:
        ad0_pub()
    except rospy.ROSInterruptException: pass
