#! /usr/bin/env python

import rospy
from mir_control.mir100_class import MiR100
from ur5e_2f85.ur5e_class import UR5e
from ur5e_2f85.robotiq_2f85_class import Robotiq2f85
from pprint import pprint

def main():
    # init node
    rospy.init_node('ur5e_2f85_python_node', anonymous=True)
    # get robot namespace and prefixes
    if rospy.has_param("/robot_namespace"):
        namespace = rospy.get_param("/robot_namespace")
    else:
        namespace = ""
    if rospy.has_param("/robot_arm_namespace"):
        arm_namespace = rospy.get_param("/robot_arm_namespace")
    else:
        arm_namespace = ""

    # init robot arm
    ur5e_arm = UR5e("arm","pilz_industrial_motion_planner","LIN")
    # go home
    ur5e_arm.set_named_pose("arm_pose1")
    # init robot gripper
    gripper = Robotiq2f85(max_gap=0.062)

    # arm poses
    pose_list = [
        "entry1",
        "pickup1",
        "entry1",
        "entry2",
        "place2",
        "entry2",
        "pickup2",
        "entry2",
        "entry1",
        "place1",
        "entry1"
    ]

    for idx, pose in enumerate(pose_list):
        joint_state = ur5e_arm.teacher_get_pose(pose)
        # if "entry" in pose:
        #     if (idx <len (pose_list) -1) and ("entry" in pose_list[idx+1]):
        #         ur5e_arm.move_j(joint_state) 
        #         continue
        ur5e_arm.move_l(joint_state)
        if "pickup" in pose:
            gripper.close()
        if "place" in pose:
            gripper.open()

    rospy.spin()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)
    except rospy.ROSInterruptException:
        pass