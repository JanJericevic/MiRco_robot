#! /usr/bin/env python

import rospy
from mir_control.mir100_class import MiR100
from mir_rest_api.api import MirRestApi
from ur5e_2f85.ur5e_class import UR5e
from ur5e_2f85.robotiq_2f85_class import Robotiq2f85
from pprint import pprint

def main():
    # init node
    rospy.init_node('moca_robot_python_node', anonymous=True)

    # get robot namespace and prefixes
    if rospy.has_param("/robot_namespace"):
        namespace = rospy.get_param("/robot_namespace")
    else:
        namespace = ""

    if rospy.has_param("/robot_base_namespace"):
        robot_base_namespace = rospy.get_param("/robot_base_namespace")
    else:
        robot_base_namespace = ""

    if rospy.has_param("/robot_arm_namespace"):
        arm_namespace = rospy.get_param("/robot_arm_namespace")
    else:
        arm_namespace = ""

    # init robot base
    mir_ip = rospy.get_param("/robot_base_ip")
    api = MirRestApi("Distributor","distributor", mir_ip)
    mir = MiR100(api)

    # init robot arm
    # ur5e_arm = UR5e("robot_arm","pilz_industrial_motion_planner","LIN")
    # go home
    # ur5e_arm.set_named_pose("home")
    # init robot gripper
    # gripper = Robotiq2f85(max_gap=0.062)

    # arm poses
    pose_list = [
        "zobnik1_entry",
        "zobnik1_pickup",
        "zobnik1_entry",
        "zobnik2_entry",
        "zobnik2_place",
        "zobnik2_entry",
        "zobnik2_pickup",
        "zobnik2_entry",
        "zobnik1_entry",
        "zobnik1_place",
        "zobnik1_entry"
    ]

    # for idx, pose in enumerate(pose_list):
    #     joint_state = ur5e_arm.teacher_get_pose(pose)
    #     # if "entry" in pose:
    #     #     if (idx <len (pose_list) -1) and ("entry" in pose_list[idx+1]):
    #     #         ur5e_arm.move_j(joint_state) 
    #     #         continue
    #     ur5e_arm.move_l(joint_state)
    #     if "pickup" in pose:
    #         gripper.close()
    #     if "place" in pose:
    #         gripper.open()

    # rospy.sleep(2)
    # for idx, pose in enumerate(pose_list):
    #     joint_state = ur5e_arm.teacher_get_pose(pose)
    #     # if "entry" in pose:
    #     #     if (idx <len (pose_list) -1) and ("entry" in pose_list[idx+1]):
    #     #         ur5e_arm.move_j(joint_state) 
    #     #         continue
    #     ur5e_arm.move_l(joint_state)
    #     if "pickup" in pose:
    #         gripper.close()
    #     if "place" in pose:
    #         gripper.open()

    rospy.spin()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)
    except rospy.ROSInterruptException:
        pass