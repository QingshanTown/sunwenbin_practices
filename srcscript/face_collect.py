#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
import sys
import easygui
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

face_detector = cv2.CascadeClassifier('/home/sunwenbin/catkin_ws/src/robot_vision/data/haar_detectors/haarcascade_frontalface_default.xml')
face_id = int(easygui.enterbox(msg="请输入当前人脸ID.",title="Warning"))
print('\n 正在拍摄人脸数据,请等待 ...')
count = 0

class image_converter:
	def __init__(self):    
		# 创建cv_bridge，声明图像的发布者和订阅者
		self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)

	def callback(self,data):
		# 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(str(e))
		
		global count
		
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)		
		faces = face_detector.detectMultiScale(gray, 1.3, 5)
		
		for (x, y, w, h) in faces:
			cv2.rectangle(cv_image, (x, y), (x+w, y+w), (255, 0, 0))
			count += 1
			cv2.imwrite('/home/sunwenbin/catkin_ws/src/sunwenbin_practices/data/0/User.' + str(face_id) + '.' + str(count) + '.jpg', gray[y: y + h, x: x + w])
		cv2.imshow('FaceDataCollect', cv_image)
			
		cv2.waitKey(1)
		
		if count >= 100:
			cv2.destroyAllWindows()

		# 再将opencv格式额数据转换成ros image格式的数据发布
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(str(e))

if __name__ == '__main__':
	try:
		# 初始化ros节点
		rospy.init_node("face_data_collect")
		rospy.loginfo("Starting face_data_collect node")
		image_converter()
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down face_data_collect node.")
		cv2.destroyAllWindows()
