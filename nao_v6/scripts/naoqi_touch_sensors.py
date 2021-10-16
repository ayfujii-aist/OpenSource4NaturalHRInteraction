#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALBroker, ALProxy, ALModule)
from naoqi_bridge_msgs.msg import Bumper
from naoqi_bridge_msgs.msg import HeadTouch
from naoqi_bridge_msgs.msg import HandTouch
from naoqi_bridge_msgs.msg import ChestButtonPressed

class NaoqiTouchSensors(NaoqiNode):
    #This should be treated as a constant
    NODE_NAME = "nao_touch_sensors"

    def __init__( self ):

        NaoqiNode.__init__( self, self.NODE_NAME )

        self.proxy = self.get_proxy("ALMemory")
        bumper_pub = rospy.Publisher('/nao_robot/naoqi_driver/bumper', Bumper, queue_size=10)
        head_pub = rospy.Publisher('/nao_robot/naoqi_driver/head_touch', HeadTouch, queue_size=10)
        hand_pub = rospy.Publisher('/nao_robot/naoqi_driver/hand_touch', HandTouch, queue_size=10)
        chest_pub = rospy.Publisher('/nao_robot/naoqi_driver/chest_touch', ChestButtonPressed, queue_size=10)
        bumper = Bumper()
        head_touch = HeadTouch()
        hand_touch = HandTouch()
        chest_button = ChestButtonPressed()

        p_rb = None
        p_lb = None
        p_ft = None
        p_mt = None
        p_rt = None
        p_hrb = None
        p_hrl = None
        p_hrr = None
        p_hlb = None
        p_hll = None
        p_hlr = None
        p_cb = None

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rb = self.proxy.getData("RightBumperPressed")
            if rb != p_rb:
                bumper.bumper = 0
                bumper.state = rb
                bumper_pub.publish(bumper)
                p_rb = rb

            lb = self.proxy.getData("LeftBumperPressed")
            if lb != p_lb:
                bumper.bumper = 1
                bumper.state = lb
                bumper_pub.publish(bumper)
                p_lb = lb

            ft = self.proxy.getData("FrontTactilTouched")
            if ft != p_ft:
                head_touch.button = 1
                head_touch.state = ft
                head_pub.publish(head_touch)
                p_ft = ft

            mt = self.proxy.getData("MiddleTactilTouched")
            if mt != p_mt:
                head_touch.button = 2
                head_touch.state = mt
                head_pub.publish(head_touch)
                p_mt = mt

            rt = self.proxy.getData("RearTactilTouched")
            if rt != p_rt:
                head_touch.button = 3
                head_touch.state = rt
                head_pub.publish(head_touch)
                p_rt = rt

            hrb = self.proxy.getData("HandRightBackTouched")
            if hrb != p_hrb:
                hand_touch.hand = 0
                hand_touch.state = hrb
                hand_pub.publish(hand_touch)
                p_hrb = hrb

            hrl = self.proxy.getData("HandRightLeftTouched")
            if hrl != p_hrl:
                hand_touch.hand = 1
                hand_touch.state = hrl
                hand_pub.publish(hand_touch)
                p_hrl = hrl

            hrr = self.proxy.getData("HandRightRightTouched")
            if hrr != p_hrr:
                hand_touch.hand = 2
                hand_touch.state = hrr
                hand_pub.publish(hand_touch)
                p_hrr = hrr

            hlb = self.proxy.getData("HandLeftBackTouched")
            if hlb != p_hlb:
                hand_touch.hand = 3
                hand_touch.state = hlb
                hand_pub.publish(hand_touch)
                p_hlb = hlb

            hll = self.proxy.getData("HandLeftLeftTouched")
            if hll != p_hll:
                hand_touch.hand = 4
                hand_touch.state = hll
                hand_pub.publish(hand_touch)
                p_hll = hll

            hlr = self.proxy.getData("HandLeftRightTouched")
            if hlr != p_hlr:
                hand_touch.hand = 5
                hand_touch.state = hlr
                hand_pub.publish(hand_touch)
                p_hlr = hlr

            cb = self.proxy.getData("ChestButtonPressed")
            if cb != p_cb:
                chest_button.state = cb
                chest_pub.publish(chest_button)
                p_cb = cb


if __name__ == '__main__':
    node = NaoqiTouchSensors()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
