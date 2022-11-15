#!/usr/bin/env python
#encoding:utf-8
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
class Obstacle():
    def __init__(self):
        self.obstacle()
    def get_scan(self):
        msg = rospy.wait_for_message("scan", LaserScan)    #订阅节点/scan信息
        self.scan_filter = []
        i= 320                          #320为雷达正前方数据，如需其他角度，修改此参数即可
        rospy.loginfo('arrarylen: %f', len(msg.ranges))
        self.scan_filter.append(msg.ranges[i])  #设置订阅节点所需提取数据。
    def obstacle(self):
        self.twist = Twist()
        while not rospy.is_shutdown():
            self.get_scan()
            rospy.loginfo('distance(m) : %f', min(self.scan_filter)) #将所需内容发送至终端
def main():
    rospy.init_node('turtlebot_scan')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
