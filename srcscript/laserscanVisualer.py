#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class laserscanvisual:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan,self.lasercallback,queue_size = 1)
        self.laserscan_pub = rospy.Publisher("/laserscanvisual", Image, queue_size=1)
        self.lidardis_pub = rospy.Publisher("/lidardis", Float32, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry,self.odomcallback,queue_size = 1)
        self.odomdir_pub = rospy.Publisher("/odomdir", Image, queue_size=1)
        self.bridge = CvBridge()
        self.angle = 0
        self.x0 = 0
        self.y0 = 0

    def odomcallback(self,data):
        frame0 = np.zeros((600, 600,3), np.uint8)
        x0 = -math.trunc(250 * math.cos((data.pose.pose.orientation.z +1.5)* 3.1416))
        y0 = math.trunc(250 * math.sin((data.pose.pose.orientation.z + 1.5)* 3.1416))
        self.x0 = x0
        self.y0 = y0
        cv2.arrowedLine(frame0, (300, 300), (x0+300, y0+300), (0, 0, 255), thickness=10, line_type=cv2.LINE_4, shift=0, tipLength=0.1)
        # 画个中心圆 
        cv2.circle(frame0, (300, 300), 10, (255, 255, 0))
        self.odomdir_pub.publish(self.bridge.cv2_to_imgmsg(frame0, "bgr8"))
        # add param
        self.angle = (data.pose.pose.orientation.z + 0.5)* 3.1416

        
    def lasercallback(self,data):
        # 设置画布为600*600像素
        frame = np.zeros((600, 600,3), np.uint8)
        # angle = data.angle_min
        angle = self.angle
        lidardis = data.ranges[0]
        if math.isinf(lidardis) == True:
            lidardis = 0.0
        for r in data.ranges:
            #如果r的值是正负无穷大，归零
            if math.isinf(r) == True:
                r = 0
            #将极坐标的信息转为直角坐标信息，只是像素的转化，不对应具体值
            #如果x中的90是正的，则顺时针显示，如果是负的，则逆时针显示。
            #x = math.trunc((r * 50.0)*math.cos(angle + (-90.0*3.1416/180.0)))
            #y = math.trunc((r * 50.0)*math.sin(angle + (-90.0*3.1416/180.0)))
            x = math.trunc((r * 50.0)*math.cos(angle + 3.1416/2))
            y = math.trunc((r * 50.0)*math.sin(angle + 3.1416/2))
            #设置限度，基本上不设置也没关系了
            if y > 600 or y < -600 or x<-600 or x>600:
                x=0
                y=0
            # 用cv2画线，位置在(300,300),和目标点，颜色是(255,0,0),线宽2
            cv2.line(frame,(300, 300),(x+300,y+300),(255,0,0),2)
            # 角度得增加
            angle= angle + data.angle_increment
        # 画个中心圆 
        cv2.circle(frame, (300, 300), 2, (255, 255, 0))
        #cv2.arrowedLine(frame, (300, 300), (-self.x0+300, -self.y0+300), (0, 0, 255), thickness=10, line_type=cv2.LINE_4, shift=0, tipLength=0.1)

        # draw arrowline
        # cv2.arrowedLine(frame, (300, 300), (300, 350), (0, 0, 255), thickness=3, line_type=cv2.LINE_4, shift=0, tipLength=0.1)
        self.laserscan_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.lidardis_pub.publish(lidardis)
        cv2.imshow('frame',frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("laser_listener")
        rospy.loginfo("Starting laser_listener node")
        laserscanvisual()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down laser_listener node."
        cv2.destroyAllWindows()
