#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def analyze(data):
    print(len(data.ranges))
    angle = data.angle_min - data.angle_max
    print(angle)
    print(angle/-data.angle_increment)

def listener():
    rospy.init_node('listener', anonymous=True)

    msg = LaserScan()
    msg = rospy.wait_for_message('b_scan', LaserScan)
    analyze(msg)

if __name__ == '__main__':
    listener()