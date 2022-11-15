#!/usr/bin/env python
#coding:utf-8
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan

def callback(data):

    newlaserscan = LaserScan()
    newlaserscan = data
    rangesarray = list(newlaserscan.ranges)
    rangenumber = 0
    while rangenumber < len(rangesarray):
        if math.isinf(rangesarray[rangenumber]) == True:
            rangesarray[rangenumber] = 0.0
        rangenumber += 1
    newlaserscan.ranges = tuple(rangesarray)
    rospy.Publisher("/scanrepublish", LaserScan, queue_size=1).publish(newlaserscan)

def laser_republisher():
    rospy.init_node('laser_republisher', anonymous=True)
    rospy.Subscriber("/scan", LaserScan,callback,queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    laser_republisher()
