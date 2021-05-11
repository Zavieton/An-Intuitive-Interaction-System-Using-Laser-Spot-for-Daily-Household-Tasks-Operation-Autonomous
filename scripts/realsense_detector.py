#!/usr/bin/env python
#coding=utf-8

"""
Created on Sun Jan 17 17:12:57 2021
@author: zavieton
using deep camera realsense
"""

import numpy as np
import cv2
import rospy
from detector_sample.msg import target_point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

weightsPath = '/home/zavieton/darknet/yolov3.weights'  # 模型权重文件
configPath = "/home/zavieton/darknet/cfg/yolov3.cfg"  # 模型配置文件
labelsPath = "/home/zavieton/darknet/data/coco.names"  # 模型类别标签文件

# weightsPath = '/home/zavieton/catkin_ws/src/detector_sample/dataset/yolov3-laser_70000.weights'  # 模型权重文件
# configPath = "/home/zavieton/catkin_ws/src/detector_sample/dataset/yolov3-laser.cfg"  # 模型配置文件
# labelsPath = "/home/zavieton/catkin_ws/src/detector_sample/dataset/laser.names"  # 模型类别标签文件


CONFIDENCE = 0.3  # 过滤弱检测的最小概率
THRESHOLD = 0.4  # 非最大值抑制阈

weightsPath2 = '/home/zavieton/darknet/yolov3-point-160_160.weights'  # 模型权重文件
configPath2 = "/home/zavieton/darknet/cfg/yolov3-voc_t.cfg"  # 模型配置文件
labelsPath2 = "/home/zavieton/darknet/data/voc_2.names"  # 模型类别标签文件

# weightsPath2 = '/home/zavieton/catkin_ws/src/detector_sample/dataset/yolov3-laser_70000.weights'  # 模型权重文件
# configPath2 = "/home/zavieton/catkin_ws/src/detector_sample/dataset/yolov3-laser.cfg"  # 模型配置文件
# labelsPath2 = "/home/zavieton/catkin_ws/src/detector_sample/dataset/laser.names"  # 模型类别标签文件

CONFIDENCE2 = 0.5  # 过滤弱检测的最小概率
THRESHOLD2 = 0.4  # 非最大值抑制阈

# target
WH = 32 * 12

net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

# point
net2 = cv2.dnn.readNetFromDarknet(configPath2, weightsPath2)
net2.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net2.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

pub = rospy.Publisher('detector_msg_info', target_point, queue_size=10)
bridge = CvBridge()

def callback(data):
    frame = bridge.imgmsg_to_cv2(data,"bgr8")
    text_1 = ''
    text_2 = ''
    label_1 = []
    label_2 = []
    point_x = []
    point_y = []
    target_x = []
    target_y = []
    target_w = []
    target_h = []
    state = 1

    target_set = 1  # 开启目标检测功能则置1
    point_set = 0  # 开启光点检测功能则置1

    blobImg = cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (WH, WH), None, True, False)
    # 拿到图片尺寸
    (H, W) = frame.shape[:2]  #720*1280

    if target_set == 1:
        net.setInput(blobImg)  # # 调用setInput函数将图片送入输入层
        # 获取网络输出层信息（所有输出层的名字），设定并前向传播
        outInfo = net.getUnconnectedOutLayersNames()  # # yolo在每个scale都有输出，outInfo是每个scale的名字信息，供net.forward使用
        layerOutputs = net.forward(outInfo)  # 得到各个输出层的、各个检测框等信息，是二维结构
        # 过滤layerOutputs
        # layerOutputs的第1维的元素内容: [center_x, center_y, width, height, objectness, N-class score data]
        # 过滤后的结果放入：
        boxes = []  # 所有边界框（各层结果放一起）
        confidences = []  # 所有置信度
        classIDs = []  # 所有分类ID

        # # 1）过滤掉置信度低的框框
        for out in layerOutputs:  # 各个输出层
            for detection in out:  # 各个框框
                # 拿到置信度
                scores = detection[5:]  # 各个类别的置信度
                classID = np.argmax(scores)  # 最高置信度的id即为分类id
                confidence = scores[classID]  # 拿到置信度

                # 根据置信度筛查
                if confidence > CONFIDENCE:
                    box = detection[0:4] * np.array([W, H, W, H])  # 将边界框放会图片尺寸
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # # 2）应用非最大值抑制(non-maxima suppression，nms)进一步筛掉
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE, THRESHOLD)  # boxes中，保留的box的索引index存入idxs
        # 得到labels列表
        with open(labelsPath, 'rt') as f:
            labels = f.read().rstrip('\n').split('\n')
        # 应用检测结果
        np.random.seed(42)
        COLORS = np.random.randint(0, 255, size=(len(labels), 3),
                                   dtype="uint8")  # 框框显示颜色，每一类有不同的颜色，每种颜色都是由RGB三个值组成的，所以size为(len(labels), 3)

        if len(idxs) > 0:
            for i in idxs.flatten():  # indxs是二维的，第0维是输出层，所以这里把它展平成1维
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                color = [int(c) for c in COLORS[classIDs[i]]]
                # frame = cv2.UMat(frame).get()
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)  # 线条粗细为2px
                text_1 = "{}: {:.4f}".format(labels[classIDs[i]], confidences[i])
                target_x.append(x)
                target_y.append(y)
                target_w.append(w)
                target_h.append(h)
                label_1.append(labels[classIDs[i]])
                cv2.putText(frame, text_1, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,
                            2)  # cv.FONT_HERSHEY_SIMPLEX字体风格、0.5字体大小、粗细2px

    if point_set == 1:
        net2.setInput(blobImg)  # # 调用setInput函数将图片送入输入层
        # 获取网络输出层信息（所有输出层的名字），设定并前向传播
        outInfo2 = net2.getUnconnectedOutLayersNames()  # # 前面的yolov3架构也讲了，yolo在每个scale都有输出，outInfo是每个scale的名字信息，供net.forward使用
        layerOutputs2 = net2.forward(outInfo2)  # 得到各个输出层的、各个检测框等信息，是二维结构。

        boxes = []  # 所有边界框（各层结果放一起）
        confidences = []  # 所有置信度
        classIDs = []  # 所有分类ID

        # # 1）过滤掉置信度低的框框
        for out in layerOutputs2:  # 各个输出层
            for detection in out:  # 各个框框
                # 拿到置信度
                scores = detection[5:]  # 各个类别的置信度
                classID = np.argmax(scores)  # 最高置信度的id即为分类id
                confidence = scores[classID]  # 拿到置信度

                # 根据置信度筛查
                if confidence > CONFIDENCE2:
                    box = detection[0:4] * np.array([W, H, W, H])  # 将边界框放会图片尺寸
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # # 2）应用非最大值抑制(non-maxima suppression，nms)进一步筛掉
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE2, THRESHOLD2)  # boxes中，保留的box的索引index存入idxs
        # 得到labels列表
        with open(labelsPath2, 'rt') as f:
            labels = f.read().rstrip('\n').split('\n')
        # 应用检测结果
        np.random.seed(20)
        COLORS = np.random.randint(0, 255, size=(len(labels), 3),
                                   dtype="uint8")  # 框显示颜色，每一类有不同的颜色，每种颜色都是由RGB三个值组成的，所以size为(len(labels), 3)

        #  cv.rectangle

        #    x1,y1 ------
        #    |          |
        #    |          |
        #    |          |
        #    --------x2,y2

        if len(idxs) > 0:
            for i in idxs.flatten():  # indxs是二维的，第0维是输出层，所以这里把它展平成1维
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)  # 线条粗细为2px
                point_x.append((x + x + w) * 0.5)
                point_y.append((y + y + w) * 0.5)
                label_2.append(labels[classIDs[i]])

                text_2 = "{}: {:.4f}".format(labels[classIDs[i]], confidences[i])
                cv2.putText(frame, text_2, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                # cv.FONT_HERSHEY_SIMPLEX字体风格、0.5字体大小、粗细2px

    if len(point_x) != 0:
        rospy.loginfo("messages: %s , Talker: point location: x=%f ,y= %f", text_2, point_x[len(point_x) - 1],
                      point_y[len(point_y) - 1])

    if len(target_x) != 0:
        target_cx = target_x[len(target_x) - 1] + 0.5 * target_w[len(target_w) - 1]
        target_cy = target_y[len(target_y) - 1] + 0.5 * target_h[len(target_h) - 1]
        rospy.loginfo("messages: %s , Talker: target location: x=%f ,y=%f", text_1, target_cx, target_cy)

    cv2.imshow("rgb_detector", frame)

    pub.publish(target_point(label_1, target_x, target_y, target_w, target_h, state, label_2, point_x, point_y))

    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()




def detector():
    # ros init
    rospy.init_node('rgb_detector', anonymous=True)
    # Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber("/camera/color/image_raw",Image,callback,queue_size= 20)  #realsense
    # rospy.Subscriber("/camera/rgb/image_raw", Image, callback, queue_size=20)  #xtion
    rospy.spin()



if __name__ == '__main__':
    detector()
