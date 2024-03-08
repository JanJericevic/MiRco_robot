#! /usr/bin/env python

import rospy
from mir_ur5e.mir100_class import MiR100
from mir_ur5e.ur5e_class import UR5e
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from pprint import pprint

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from six.moves import input


def main():

    ur5e_arm = UR5e("arm")
    # manipulator.pick_and_place()
    # del manipulator

    # info = ur5e_arm.get_info()

    # current_pose = PoseStamped()
    # current_pose = info["current_pose"]
    # new_pose = PoseStamped()
    # new_pose.header = current_pose.header
    # new_pose.header.stamp = rospy.Time.now()
    # new_pose.header.frame_id = 'base_link_inertia'

    # new_pose.pose = current_pose.pose
    # new_pose.pose.position.x = 0.29627
    # new_pose.pose.position.y = -0.1345 + 0.4
    # new_pose.pose.position.z = 0.44066

    # new_pose.pose.orientation.x = -0.706900001
    # new_pose.pose.orientation.y = 0.70731000001
    # new_pose.pose.orientation.z = -0.002511800001
    # new_pose.pose.orientation.w = -0.0011571300001

    # ur5e_arm.set_pose(new_pose)

    info = ur5e_arm.get_info(print=True)
    ur5e_arm.set_named_pose("arm_home")
    rospy.sleep(1)
    ur5e_arm.set_named_pose("arm_pickup")
    rospy.sleep(1)
    ur5e_arm.set_named_pose("arm_home")
    rospy.sleep(1)
    ur5e_arm.set_named_pose("arm_place_intermediate")
    rospy.sleep(1)
    ur5e_arm.set_named_pose("arm_place")
    rospy.sleep(1)
    ur5e_arm.set_named_pose("arm_place_intermediate")
    rospy.sleep(1)
    ur5e_arm.set_named_pose("arm_home")

    rospy.sleep(3)
    rospy.signal_shutdown("motion concluded")

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)