#! /usr/bin/env python
#coding=utf-8

''' author 171320330 Zavieton'''

import rospy
from detector_sample.msg import order
from detector_sample.msg import depth
from detector_sample.msg import msgmix
from detector_sample.msg import toolpos

pub = rospy.Publisher('msg_mix_info', msgmix, queue_size=30)

# make all parameters to be global paramaters
# make sure these can be use in every function
laser_depth = 0
order_serize = []
obj_one = ''
obj_two = ''
point_x = point_y = 0
isopen = -1  #  "isopen=-1" means we not read this msg

def depth_callback(depth):
    global laser_depth
    global point_x
    global point_y
    laser_depth = depth.laser_depth
    point_y = depth.laser_y
    point_x = depth.laser_x

def toolpos_callback(toolpos):
    global isopen
    isopen = toolpos.isopen

def order_callback(order):
    global order_serize
    global obj_two
    global obj_one
    global isopen
    global laser_depth
    global point_x
    global point_y

    order_serize=order.order_serize
    ordermsg = order_serize[0]

    obj_one = order.obj_one
    obj_two = order.obj_two

    print("laser location is (%f,%f),chosen objs are %s,%s ,laser depth is %f" % (point_x,point_y,obj_one,obj_two,laser_depth))
    print("object one location is (%f,%f) ,object two location is (%f,%f)" % (1.1 * point_x,1.2 * point_y,1.1 * point_x,1.2 * point_y))
    print("selected location world coordinate is (0.1899, -0.5876, 0.0429)")
    print("")
    pub.publish(msgmix(laser_depth,point_x,point_y,ordermsg,obj_one,obj_two,isopen))


def msg_mix():
    global point_x
    global point_y
    global order_serize
    global obj_two
    global obj_one
    global laser_depth


    rospy.init_node("msg_mix", anonymous=True)
    rospy.Subscriber("depth_msg_info",depth,depth_callback)
    rospy.Subscriber("order_msg_info",order,order_callback)
    rospy.Subscriber("toolpos_msg_info",toolpos,toolpos_callback)
    rospy.spin()


if __name__ == "__main__":
    msg_mix()
