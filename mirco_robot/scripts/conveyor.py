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
        # print(result)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def dock2marker(service_name, marker):
    try:
        docking_service = rospy.ServiceProxy(service_name, DockToMarker)
        result = docking_service(marker) 
        # print(result)

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
    docking_service_name = namespace + robot_base_namespace + "/mir_control_node/dock_to_vl_marker"
    rospy.wait_for_service(goal_service_name)
    rospy.wait_for_service(docking_service_name)

    
    # ------- DELIVERY -------
    for i in range(1):

        rospy.sleep(7)

        # pick and place sequence
        pose_list_1 = [
            "zobnik1_entry",
            "zobnik1_pickup",
            "zobnik1_entry",
            "conveyor_start",
            "conveyor_entry",
            "conveyor_place",
            "conveyor_entry",
            "conveyor_start",
            "conveyor_entry",
            "conveyor_pickup",
            "conveyor_entry",
            "conveyor_start",
            "zobnik1_entry",
            "zobnik1_place",
            "zobnik1_entry",
            "home"
        ]

        for idx, pose in enumerate(pose_list_1):
            joint_state = ur5e_arm.teacher_get_pose(pose)
            if idx >=1 and pose_list_1[idx-1] == "conveyor_start" and pose_list_1[idx] == "conveyor_entry":
                ur5e_arm.move_j(joint_state)
            elif idx >=1 and pose_list_1[idx-1] == "conveyor_entry" and pose_list_1[idx] == "conveyor_start":
                ur5e_arm.move_j(joint_state)
            else:
                ur5e_arm.move_l(joint_state)
            if "pickup" in pose:
                gripper.close()
            if "place" in pose:
                gripper.open()
            
            rospy.sleep(1)
        
        i = i+1

    # rospy.spin()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)
    except rospy.ROSInterruptException:
        pass