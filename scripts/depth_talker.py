#! /usr/bin/env python
#coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detector_sample.msg import target_point
from detector_sample.msg import depth


x = y = 0   # init x,y
point_y = ()
pub = rospy.Publisher('depth_msg_info', depth, queue_size=10)

def msg_callback(target_point):

    # rate1 = 0.66
    # rate2 = 0.66
    rate1 = rate2 = 1
    global x
    global y
    global point_y

    if target_point.state == True:  # if True ，receieve the message
        point_x = target_point.point_x
        point_y = target_point.point_y
        if point_y != ():
            x = point_x[len(point_x) - 1]
            x = x * rate1
            y = point_y[len(point_y) - 1]
            y = y * rate2

        else:
            x = y = 10

def depth_callback(data):
    global x
    global y
    global point_y

    bridge = CvBridge()
    depth_m = bridge.imgmsg_to_cv2(data, 'passthrough')
    m = int(depth_m[int(y),int(x)])

    if point_y !=():

        rospy.loginfo("%f",m)
        pub.publish(depth(m,x,y))


def depth_talker():

    rospy.init_node("depth_msg", anonymous=True)
    rospy.Subscriber('detector_msg_info', target_point,msg_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image,depth_callback)

    rospy.spin()


if __name__ == "__main__":
    depth_talker()
