#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('mir_pose_publisher', anonymous=True)
    rate = rospy.Rate(20)

    listener = tf.TransformListener()
    pub = rospy.Publisher('mir_pose', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"

            pose = Pose()
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]

            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = pose

            pub.publish(pose_stamped)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()