#!/usr/bin/env python
#encoding:utf-8
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

def callback(data):
    # 设置画布为600*600像素
    frame = np.zeros((600, 600,3), np.uint8)
    angle = data.angle_min
    for r in data.ranges:
        #change infinite values to 0
        #如果r的值是正负无穷大，归零
        if math.isinf(r) == True:
            r = 0
        #convert angle and radius to cartesian coordinates
        #这里就是将极坐标的信息转为直角坐标信息，只是像素的转化，不对应具体值
        #如果x中的90是正的，则顺时针显示，如果是负的，则逆时针显示。
        x = math.trunc((r * 50.0)*math.cos(angle + (-90.0*3.1416/180.0)))
        y = math.trunc((r * 50.0)*math.sin(angle + (-90.0*3.1416/180.0)))

        #set the borders (all values outside the defined area should be 0)
        #设置限度，基本上不设置也没关系了
        if y > 600 or y < -600 or x<-600 or x>600:
            x=0
            y=0
       # print "xy:",x,y
		# 用CV2画线，位置在(300,300),和目标点，颜色是(255,0,0),线宽2
        cv2.line(frame,(300, 300),(x+300,y+300),(255,0,0),2)
		# 角度得增加
        angle= angle + data.angle_increment
        # 画个中心圆 
        cv2.circle(frame, (300, 300), 2, (255, 255, 0))
        rospy.Publisher("laserscanvisual", Image, queue_size=10).publish(frame)
        cv2.imshow('frame',frame)
        cv2.waitKey(1)


def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan,callback,queue_size = 1)
    
    rospy.spin()

if __name__ == '__main__':
    laser_listener()
    
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

