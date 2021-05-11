#! /usr/bin/env python
#coding=utf-8

'''
Author: zaveiton 171320330
2021.04
'''

import rospy
from kinova_msgs.msg import FingerPosition
from detector_sample.msg import toolpos

pub = rospy.Publisher('toolpos_msg_info',toolpos,queue_size=10)

finger1 = -1
finger2 = -1
finger3 = -1


def finger_callback(FingerPosition):
    global finger1
    global finger2
    global finger3
    finger1 = FingerPosition.finger1
    finger2 = FingerPosition.finger2
    finger3 = FingerPosition.finger3
    if (finger1 >= 3000) and (finger2 >= 3000):
        isopen = 0    # close
    else:
        isopen = 1    # open
    rospy.loginfo("finger is %f, %f, %f", finger1, finger2, finger3)
    rospy.loginfo("isopen is %f", isopen)

    pub.publish(toolpos(finger1,finger2,finger3,isopen))


def depth_talker():

    rospy.init_node("toolspos", anonymous=True)
    rospy.Subscriber("/j2n6a300_driver/out/finger_position", FingerPosition, finger_callback)                # 订阅kinova机械臂手指的信息
    rospy.spin()


if __name__ == "__main__":
    depth_talker()
