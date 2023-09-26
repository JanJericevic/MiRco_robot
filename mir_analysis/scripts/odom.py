#!/usr/bin/env python

"""
when using rostopic echo -p --noarr /odom:


message received time in nanoseconds,
header:seq,
header:stamp:secs & header:stamp:nsecs
header:frame_id,
child_frame_id,
pose info,
twist info

no covariance arrays because of --noarr
"""


import rospy
from nav_msgs.msg import Odometry

def analyze(msg):
    print(msg.twist.twist.linear.x)
    if msg.twist.twist.linear.x > 2:
        print("TARGET VELOCITY REACHED! VELOCITY = %s" %(msg.twist.twist.linear.x))
        rospy.signal_shutdown("target velocity reached")

def main():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/odom", Odometry, analyze)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass