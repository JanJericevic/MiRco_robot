#! /usr/bin/env python

import rospy
import rospkg
from mir_control.mir100_class import MiR100
from mir_control.srv import *
from mir_rest_api.api import MirRestApi
from ur5e_2f85.ur5e_class import UR5e
from ur5e_2f85.robotiq_2f85_class import Robotiq2f85
from pprint import pprint

def send2goal(service_name, target_goal):
    try:
        goal_service = rospy.ServiceProxy(service_name, GoToGoal)
        result = goal_service(target_goal) 
        print(result)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():
    # init node
    rospy.init_node('mirco_delivery_node', anonymous=True)

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

    # init robot arm
    rospack = rospkg.RosPack()
    package = "mirco_robot"
    file_name = rospack.get_path(package) + "/config/ur5e_saved_poses.yml" 
    ur5e_arm = UR5e("robot_arm", pose_file = file_name)
    # go home
    ur5e_arm.set_named_pose("home")

    # init robot gripper
    gripper = Robotiq2f85(max_gap=0.062)

    # ros service clients for controling mir
    goal_service_name = namespace + robot_base_namespace + "/mir_control_node/send_to_goal"
    rospy.wait_for_service(goal_service_name)
    
    # ------- DELIVERY -------
    rospy.sleep(3)
    # send mir to goal1
    # send2goal(goal_service_name,"start")
    # send2goal(goal_service_name,"delivery")

    # pick and place sequence
    pose_list = [
        "zobnik1_pickup",
        "zobnik1_entry",
        "conveyor_start",
        "conveyor_entry_testing",
        "conveyor_place_testing",
        "conveyor_entry_testing",
        "conveyor_start"
    ]

    for idx, pose in enumerate(pose_list):
        print(pose)
        joint_state = ur5e_arm.teacher_get_pose(pose)
        if idx >=1 and pose_list[idx-1] == "conveyor_start" and pose_list[idx] == "conveyor_entry_testing":
            ur5e_arm.move_j(joint_state)
        elif idx >=1 and pose_list[idx-1] == "conveyor_entry_testing" and pose_list[idx] == "conveyor_start":
            ur5e_arm.move_j(joint_state)
        else:
            ur5e_arm.move_l(joint_state)
        if "pickup" in pose:
            gripper.close()
        if "place" in pose:
            gripper.open()

    ur5e_arm.set_named_pose("home")
    rospy.sleep(1)

    # send2goal(goal_service_name,"end")

    # rospy.spin()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)
    except rospy.ROSInterruptException:
        pass