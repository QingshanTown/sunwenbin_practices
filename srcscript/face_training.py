#!/usr/bin/env python
#coding:utf-8

import numpy as np
from PIL import Image
import os
import cv2

path = '/home/sunwenbin/catkin_ws/src/sunwenbin_practices/data/0'

recognizer = cv2.face.createLBPHFaceRecognizer()
detector = cv2.CascadeClassifier("/home/sunwenbin/catkin_ws/src/robot_vision/data/haar_detectors/haarcascade_frontalface_default.xml")

def getImagesAndLabels(path):
	imagePaths = [os.path.join(path, f) for f in os.listdir(path)]
	faceSamples = []
	ids = []
	for imagePath in imagePaths:
		PIL_img = Image.open(imagePath).convert('L')   
		img_numpy = np.array(PIL_img, 'uint8')
		id = int(os.path.split(imagePath)[-1].split(".")[1])
		faces = detector.detectMultiScale(img_numpy)
		for (x, y, w, h) in faces:
			faceSamples.append(img_numpy[y:y + h, x: x + w])
			ids.append(id)
	return faceSamples, ids

print('正在训练人脸模型数据,请等待...')
faces, ids = getImagesAndLabels(path)
recognizer.train(faces, np.array(ids))

recognizer.save(r'/home/sunwenbin/catkin_ws/src/sunwenbin_practices/data/trainer.yml')
print("{0}组人脸模型数据训练完成,程序退出".format(len(np.unique(ids))))
