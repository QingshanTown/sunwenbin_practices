#!/usr/bin/env python
#coding:utf-8
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from covariances import \
     ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2

class imu_odom_republish:
    def __init__(self):    
        self.imu_sub = rospy.Subscriber("/imu", Imu,self.imucallback,queue_size = 1)
        self.imu_pub = rospy.Publisher("/imuwithcovariances", Imu, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry,self.odomcallback,queue_size = 1)
        self.odom_pub = rospy.Publisher("/odomwithcovariances", Odometry, queue_size=1)


    def imucallback(self,data):
        msg = Imu()
        msg = data
        msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        msg.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        msg.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
        self.imu_pub.publish(msg)
        # rospy.loginfo("Successfully republish Imu message")
        
    def odomcallback(self,data):
        msg = Odometry()
        msg = data
        if msg.twist.twist.linear.x == 0 and msg.twist.twist.angular.z == 0:
            msg.pose.covariance = ODOM_POSE_COVARIANCE2
            msg.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            msg.pose.covariance = ODOM_POSE_COVARIANCE
            msg.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_pub.publish(msg)
        # rospy.loginfo("Successfully republish Odometry message")
        


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("imu_odom_republisher")
        rospy.loginfo("Starting imu_odom_republisher node")
        imu_odom_republish()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down laser_listener node."