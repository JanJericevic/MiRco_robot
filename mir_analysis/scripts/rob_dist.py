#!/usr/bin/env python

import rospy
from pprint import pprint as pp
from scipy import io
import numpy as np
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def end():
    print("No message received for 5 seconds")
    print("Saving variables to .mat file")
    
    pos = np.column_stack((pos_x,pos_y,pos_z))
    vel = np.column_stack((vel_x,ang_z))

    mdic = {"pos": pos, "vel": vel, "ang_min": ang_min, "ang_max": ang_max,"ang_inc": ang_inc,"ran_min": ran_min,"ran_max":ran_max, "ranges": ranges}
    io.savemat("bag_file_test5.2_150cm_s.mat", mdic,oned_as='column')

    rospy.signal_shutdown("finished")

def timeout():
    print("No message received for 10 seconds")
    rospy.signal_shutdown("timeout")
    

def callback_odom(msg):
    print("Message received")

    global pos_x, pos_y, pos_z
    global vel_x, ang_z

    global timer
    timer.cancel()
    timer = threading.Timer(5,end)
    timer.start()

    pos_x = np.append(pos_x, msg.pose.pose.position.x)
    pos_y = np.append(pos_y, msg.pose.pose.position.y)
    pos_z = np.append(pos_z, msg.pose.pose.position.z)

    vel_x = np.append(vel_x, msg.twist.twist.linear.x)
    ang_z = np.append(ang_z, msg.twist.twist.angular.z)

def callback_scan(msg):
    global ang_min, ang_max, ang_inc, t_inc
    global ran_min, ran_max, ranges

    ang_min = msg.angle_min
    ang_max = msg.angle_max
    ang_inc = msg.angle_increment

    ran_min = msg.range_min
    ran_max = msg.range_max
    ranges = np.vstack((ranges,np.array(msg.ranges)))

def main():
    #odometry
    global pos_x, pos_y, pos_z
    global vel_x, ang_z

    pos_x = np.array([])
    pos_y = np.array([])
    pos_z = np.array([])

    vel_x = np.array([])
    ang_z = np.array([])

    #laser scan
    global ang_min, ang_max, ang_inc, t_inc
    global ran_min, ran_max, ranges

    ranges = np.empty(shape=(541,))

    #nodes
    rospy.init_node('rosbag_listener', anonymous=True)

    global timer
    timer = threading.Timer(10,timeout) 
    timer.start()

    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/f_scan", LaserScan, callback_scan)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass