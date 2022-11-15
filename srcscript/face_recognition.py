#!/usr/bin/env python
#coding:utf-8

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

recognizer = cv2.face.createLBPHFaceRecognizer()
recognizer.load('/home/sunwenbin/catkin_ws/src/sunwenbin_practices/data/trainer.yml')
faceCascade = cv2.CascadeClassifier("/home/sunwenbin/catkin_ws/src/robot_vision/data/haar_detectors/haarcascade_frontalface_default.xml")
font = cv2.FONT_HERSHEY_SIMPLEX

idnum = 0
names = ['sunwenbin', 'zhangxinning','Walt']
minW = 0.1*640
minH = 0.1*480

class image_converter:
	def __init__(self):    
		# 创建cv_bridge，声明图像的发布者和订阅者
		self.image_pub = rospy.Publisher("face_recognition_image", Image, queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)

	def callback(self,data):
		# 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(str(e))

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		faces = faceCascade.detectMultiScale(
			gray,
			scaleFactor=1.2,
			minNeighbors=5,
			minSize=(int(minW), int(minH))
		)
		
		for (x, y, w, h) in faces:
			cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
			idnum, confidence = recognizer.predict(gray[y:y+h, x:x+w])

			if confidence < 100:
				idnum = names[idnum]
				confidence = "{0}%".format(round(100 - confidence))
			else:
				idnum = "unknown"
				confidence = "{0}%".format(round(100 - confidence))

			cv2.putText(cv_image, str(idnum), (x+5, y-5), font, 1, (0, 0, 255), 1)
			cv2.putText(cv_image, str(confidence), (x+5, y+h-5), font, 1, (0, 0, 0), 1)

		#cv2.imshow('camera', cv_image)
        
		#cv2.waitKey(1)

		# 再将opencv格式额数据转换成ros image格式的数据发布
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(str(e))

if __name__ == '__main__':
	try:
		# 初始化ros节点
		rospy.init_node("face_recognition")
		rospy.loginfo("Starting face_rocognition node")
		image_converter()
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down face_recognition node.")
		cv2.destroyAllWindows()
