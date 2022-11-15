#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class robot_avoidance:
    def __init__(self):
        #定义基本的发布者和订阅者
        self.laserscan_sub = rospy.Subscriber("/scanrepublish", LaserScan, self.callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def callback(self,data):
        lidardistance = data.ranges[0]
        # lidardistance = data.ranges[320]
        self.twist_calculate(lidardistance)

    def twist_calculate(self,distance):
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        if distance > 0.15:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
            print ("keep running")
        else:
            self.twist.linear.x = -0.4
            self.twist.angular.z = 0.4
            print ("avoidance")
        self.cmd_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        # init ROS node
        rospy.init_node("robot_avoidance")
        robot_avoidance()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down robot_avoidance node.")
        cv2.destroyAllWindows()
