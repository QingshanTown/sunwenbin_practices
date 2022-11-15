#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class robot_follow:
    def __init__(self):
        #定义基本的发布者和订阅者
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback, queue_size=1)
        self.image_pub = rospy.Publisher("face_image", Image, queue_size=1)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # 获取haar特征的级联表的XML文件，文件路径在launch文件中传入
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")

        # 使用级联表初始化haar特征检测器
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)

        # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (50, 255, 50)

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as e:
            print (e)
        # 创建灰度图像
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 创建平衡直方图，减少光线影响
        grey_image = cv2.equalizeHist(grey_image)
        # 尝试检测人脸
        faces_result = self.detect_face(grey_image)
        # 绘制人脸范围以及中心点
        height, width = frame.shape[0:2]
        screen_center = width / 2
        offset = 50
        # 在opencv的窗口中框出所有人脸区域
        if len(faces_result) > 0:
            x, y, w, h = faces_result[0]
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), self.color, 2)
            center_x = int (x + w/2)
            center_y = int (y + h/2)
            cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
            self.twist_calculate(screen_center,center_x,offset)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(img_msg)

        except CvBridgeError as e:
            print (e)

    def detect_face(self, input_image):
        # 首先匹配正面人脸的模型
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(input_image,
                    self.haar_scaleFactor,
                    self.haar_minNeighbors,
                    cv2.CASCADE_SCALE_IMAGE,
                    (self.haar_minSize, self.haar_maxSize))
        # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
        if len(faces) == 0 and self.cascade_2:
            faces = self.cascade_2.detectMultiScale(input_image,
                    self.haar_scaleFactor,
                    self.haar_minNeighbors,
                    cv2.CASCADE_SCALE_IMAGE,
                    (self.haar_minSize, self.haar_maxSize))
        return faces

    def twist_calculate(self,screen_center,center_x,offset):
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        if center_x < screen_center - offset:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.5
            print ("turn left")
        elif screen_center - offset <= center_x <= screen_center + offset:
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0
            print ("keep")
        elif center_x > screen_center + offset:
            self.twist.linear.x = 0.1
            self.twist.angular.z = -0.5
            print ("turn right")
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            print ("stop")
        self.pub_cmd.publish(self.twist)

if __name__ == '__main__':
    try:
        # init ROS node
        rospy.init_node("robot_follow")
        rospy.loginfo("Starting Robot Follow node")
        robot_follow()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()
