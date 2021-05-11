#!/usr/bin/env python
#coding=utf-8

"""
Created on Sun Jan 17 20:42:50 2021
@author: zavieton
"""

import rospy
from detector_sample.msg import target_point
from svmutil import *
import numpy as np
from detector_sample.msg import order


count_list = []
label_list = []
activition = 0
forward = 1

pub = rospy.Publisher('order_msg_info', order, queue_size=10)


def cal_distance(x1,y1,x,w,y,h):
    centerx = x + 0.5*w
    centery = y + 0.5*h
    distance = np.sqrt(np.square(x1 - centerx) + np.square(y1 - centery))
    return distance


def callback(target_point):
    # 回调函数 callback
    global count_list
    global activition
    global label_list
    global forward
    detect_len = 26  #detecte length
    object_label = ''
    if target_point.state == True:  # if True ，receieve the message
        target_x = target_point.target_x
        target_y = target_point.target_y
        target_w = target_point.target_w
        target_h = target_point.target_h
        point_x = target_point.point_x
        point_y = target_point.point_y
        label_target = target_point.target_label

        # lable get function
        if target_x != ():
            distance = 999999999999
            for i in range(len(target_x)):
                if point_y !=():
                    center_distance = cal_distance(point_x[len(point_x)-1],point_y[len(point_y)-1],target_x[i],target_w[i],target_y[i],target_h[i])
                    if point_y[len(point_y)-1] < target_y[i]+ target_h[i] and  point_y[len(point_y)-1] > target_y[i]:
                        if point_x[len(point_x)-1] < target_x[i]+ target_w[i] and  point_x[len(point_x)-1] > target_x[i]:
                            if distance > center_distance:
                                object_label = label_target[i]
                                distance = center_distance

        if len(target_point.label_point) != 0 and activition == 0 and forward == 1:
            activition = 1
            forward = 0

        if activition == 1:
            if len(target_point.label_point) != 0 and len(count_list) <= detect_len:
                count_list.append(1)
                label_list.append(object_label)

            elif len(target_point.label_point) == 0 and len(count_list) <= detect_len:
                count_list.append(0)
                label_list.append('')
            elif len(count_list) > detect_len:
                activition = 0
                count_list = []
                label_list = []
        if len(count_list) == 0 and len(target_point.label_point) == 0:
            forward = 1

        if len(count_list) == detect_len:
            # print(count_list)
            # print(label_list)  # label list is including the object label

            series = dict(zip(range(1,detect_len+1),count_list))
            series = [series]
            p_label, p_acc, p_val = svm_predict([], series, model,'-q')

            # msg mix
            forward_obj = back_obj = 'NULL'

            for i in range(len(label_list)):
                if label_list[i] != '':
                    forward_obj = label_list[i]

                if label_list[len(label_list) - i - 1] != '':
                    back_obj = label_list[len(label_list) - i - 1]

            if p_label == [1.0] or p_label == [3.0]:
                if p_label == [1.0]:
                    rospy.loginfo("点按，选择物体：%s", back_obj)
                elif p_label == [3.0]:
                    rospy.loginfo("长按，选择物体：%s", back_obj)

                pub.publish(order(p_label, back_obj, forward_obj))

            elif p_label == [2.0] or p_label == [4.0] or p_label == [5.0] or p_label == [6.0]:
                if p_label == [2.0]:
                    rospy.loginfo("点按两次，选择物体 %s 与 %s", back_obj, forward_obj)
                elif p_label == [4.0]:
                    rospy.loginfo("长按后点选 , 选择物体 %s 与 %s", back_obj, forward_obj)
                elif p_label == [5.0]:
                    rospy.loginfo("点选后长按 , 选择物体 %s 与 %s", back_obj, forward_obj)
                elif p_label == [6.0]:
                    rospy.loginfo("长按两次 , 选择物体 %s 与 %s", back_obj, forward_obj)

                pub.publish(order(p_label,back_obj,forward_obj))



def process_listener():
    print("*************  Model is created Successful!  **************")
    # ros init
    rospy.init_node('process_listener', anonymous=True)
    # Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber('detector_msg_info', target_point, callback)
    rospy.spin()



if __name__ == '__main__':
    y, x = svm_read_problem('/home/zavieton/catkin_ws/src/detector_sample/dataset/trainv32.txt')
    model = svm_train(y, x)
    parameter = '-c 2.0 -g 0.5 -t 2'
    process_listener()
