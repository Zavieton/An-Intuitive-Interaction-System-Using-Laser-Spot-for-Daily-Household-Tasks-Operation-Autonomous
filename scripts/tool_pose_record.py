#!/usr/bin/env python
# -*-coding:utf-8-*-
'''
' author:ly
' description: 收集末端笛卡尔轨迹和各关节数据
'
'
'''
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
import pandas as pd
import time
import math
import argparse
from time import sleep
# import threading 尝试增加多线程的，没用上

from dmp.srv import *
from dmp.msg import *

import matplotlib
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from geometry_msgs.msg import PoseStamped
# from my_pcl_tutorial.msg import jacos
#from darknet_ros_msgs.msg import ObjActIntention
from std_msgs.msg import String
from kinova_msgs.msg import FingerPosition
from kinova_msgs.msg import JointTorque

# 1.在ubuntu中汉字为框框，需要加如下头文件
#import sys
#import importlib
#importlib.reload(sys)

global_finger_pose = [0,0,0] 
global_record_coordinate = []
global_JointTorque = [0.0,0.0,0.0,0.0,0.0,0.0]

def PlotPicture(first_plot,second_plot):

    # 为了能显示中文
    plt.rcParams['font.sans-serif']=['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 设置绘图窗口的尺寸
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')  # 可进行多图绘制

    # 设置图表标题并给坐标轴加上标签
    ax.set_xlabel('X轴')
    ax.set_ylabel('Y轴')
    ax.set_zlabel('Z轴')
    ax.set_title('DMP轨迹点泛化前后对比')

    # 画散点图 edgecolors=None删掉点的轮廓， s=5点的尺寸， c=(0, 0, 0.8)颜色RGB值（0-1） cmap=plt.cm.Blues, c=point_numbers 颜色渐变 marker='o'点型
    # point_numbers = list(range(len(first_plot)))
    for x in first_plot:
        t1 = ax.scatter(x[0], x[1], x[2], edgecolors=None, s=5,color=(0, 0, 0.9), marker='o')  # 用散点函数画点matplotlib.pyplot.scatter(x, y, s=点的大小, c=None, marker=None, cmap=None, norm=None, vmin=None, vmax=None, alpha=None, linewidths=None, verts=None, edgecolors=None, *, data=None, **kwargs)
    for y in second_plot:
        t2 = ax.scatter(y[0], y[1], y[2], edgecolors=None, s=5,color=(0, 0.8, 0), marker='x')  # 用散点函数画点matplotlib.pyplot.scatter(x, y, s=None, c=None, marker=None, cmap=None, norm=None, vmin=None, vmax=None, alpha=None, linewidths=None, verts=None, edgecolors=None, *, data=None, **kwargs) 

    # 设置标签
    lgnd = plt.legend([t1,t2],["原轨迹","新轨迹"], bbox_to_anchor=(1.05, 1), loc=1)
        # lgnd = plt.legend(["新轨迹"], bbox_to_anchor=(1.05, 1), loc=1)
    # 设置刻度标记的大小
    plt.tick_params(axis="both", which='major', labelsize=10)

    plt.show()


def QuaternionNorm(Q_raw):                                   # 正则化四元数
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    if qnorm != 0:
        qx_ = qx_temp/qnorm
        qy_ = qy_temp/qnorm
        qz_ = qz_temp/qnorm
        qw_ = qw_temp/qnorm
        Q_normed_ = [qx_, qy_, qz_, qw_]

    return Q_normed_

    
def Quaternion2EulerXYZ(Q_raw):                              # 四元数转欧拉
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]

    return EulerXYZ_


def JointTorque_pose(jointtorqueposition):
    # print fingerposition
    global global_JointTorque
    # print(jointtorqueposition)
    JointTorque_pose = [0.0,0.0,0.0,0.0,0.0,0.0]
    JointTorque_pose[0] = jointtorqueposition.joint1
    JointTorque_pose[1] = jointtorqueposition.joint2
    JointTorque_pose[2] = jointtorqueposition.joint3
    JointTorque_pose[3] = jointtorqueposition.joint4
    JointTorque_pose[4] = jointtorqueposition.joint5
    JointTorque_pose[5] = jointtorqueposition.joint6

    global_JointTorque = JointTorque_pose


def finger_pose(fingerposition):
    # print fingerposition
    global global_finger_pose
    finger_pose = [0,0,0]
    finger_pose[0] = fingerposition.finger1
    finger_pose[1] = fingerposition.finger2
    finger_pose[2] = fingerposition.finger3
    global_finger_pose = finger_pose

def tool_pose(posestamped):                                                 # 订阅的kinova末端的坐标

    global global_finger_pose
    # print "--------------------------已收到 KINOVA 发布过来的坐标消息--------------------------"                                                   # 开了以后太能叭叭了
    # tool_pose_C_point = [0.0,0.0,0.0]                                     # 机械臂的3维坐标全局变量 Cartesian
    # tool_pose_C_point[0] = posestamped.pose.position.x
    # tool_pose_C_point[1] = posestamped.pose.position.y
    # tool_pose_C_point[2] = posestamped.pose.position.z
    tool_pose_Q_point = [0.0,0.0,0.0,0.0]
    tool_pose_Q_point[0] = posestamped.pose.orientation.x
    tool_pose_Q_point[1] = posestamped.pose.orientation.y
    tool_pose_Q_point[2] = posestamped.pose.orientation.z
    tool_pose_Q_point[3] = posestamped.pose.orientation.w

    EulerXYZ = Quaternion2EulerXYZ(tool_pose_Q_point)                       # 四元数到欧拉的变换函数 posestamped.pose.position:机械臂的笛卡尔xyz posestamped.pose.orientation:机械臂姿态四元数
    tool_record_coordinate = []
    tool_record_coordinate = [posestamped.pose.position.x,posestamped.pose.position.y,posestamped.pose.position.z,EulerXYZ[0],EulerXYZ[1],EulerXYZ[2],global_finger_pose[0],global_finger_pose[1],global_finger_pose[2],global_JointTorque[0],global_JointTorque[1],global_JointTorque[2],global_JointTorque[3],global_JointTorque[4],global_JointTorque[5]]

    
    print ("X: ", posestamped.pose.position.x,"Y: ", posestamped.pose.position.y, "Z: ", posestamped.pose.position.z, "Theta X: ",EulerXYZ[0], "Theta Y: ", EulerXYZ[1], "Theta Z: ",EulerXYZ[2],"finger1: ", global_finger_pose[0], "finger2: ", global_finger_pose[1], "finger3: ", global_finger_pose[2],"joint1: ",global_JointTorque[0],"joint2: ",global_JointTorque[1],"joint3: ",global_JointTorque[2],"joint4: ",global_JointTorque[3],"joint5: ",global_JointTorque[4],"joint6: ",global_JointTorque[5])
    output = open("//home//penghaoqi//catkin_ws//src//kinova-ros//kinova_driver//trajectory//record.txt","a")                                                      # "a"是追加 'w'是覆盖
    # new_record_coordinate = str(list(np.reshape(np.asarray(record_coordinate),(1,np.size(record_coordinate)))[0]))[1:-1]                                  # 这个操作挺牛逼，就是没实现效果，存着留着以后看
    new_record_coordinate = " ".join(str(i) for i in tool_record_coordinate)     # 将每个元素用空格分隔开
    output.write(new_record_coordinate)

    output.write("\n")
    output.close()

    return 0


if __name__ == '__main__':
    rospy.init_node('record_trajectory',anonymous=True)
    rospy.Subscriber("/j2n6s300_driver/out/tool_pose", PoseStamped, tool_pose);                            # 订阅kinova机械臂当前的信息
    rospy.Subscriber("/j2n6s300_driver/out/finger_position", FingerPosition, finger_pose);                 # 订阅kinova机械臂手指的信息
    rospy.Subscriber("/j2n6s300_driver/out/joint_torques", JointTorque, JointTorque_pose);
    rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
        # print("loop")
        # rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

