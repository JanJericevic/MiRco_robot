#! /usr/bin/env python

import rospy
from mir_ur5e.mir100_class import MiR100
from mir_ur5e.ur5e_class import UR5e
from mir_ur5e.robotiq_2f85_class import Robotiq2f85
from mir_ur5e.pose_teacher import PoseTeacher
from mir_ur5e.srv import *
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, Pose
from pprint import pprint

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from six.moves import input


def main():
    # init node
    rospy.init_node('ur5e_2f85_python_node', anonymous=True)
    # get robot namespace and prefixes
    if rospy.has_param("/robot_namespace_prefix"):
        namespace = rospy.get_param("/robot_namespace_prefix")
    else:
        namespace = ""

    # wait for pose teacher services
    save_pose_srv_name = rospy.get_namespace() + "/pose_teacher_node/save_arm_pose"
    get_pose_srv_name = rospy.get_namespace() + "/pose_teacher_node/get_arm_pose"
    rospy.wait_for_service(save_pose_srv_name)
    rospy.wait_for_service(get_pose_srv_name)

    # create service clients
    save_pose_srv = rospy.ServiceProxy(save_pose_srv_name, SavePose)
    get_pose_srv = rospy.ServiceProxy(get_pose_srv_name, GetPose)

    #init robot arm and gripper
    ur5e_arm = UR5e("arm","pilz_industrial_motion_planner","LIN")
    rospy.sleep(1)

    gripper = Robotiq2f85(max_gap=0.062)
    rospy.sleep(2)

    #go home
    ur5e_arm.group.set_planner_id("PTP")
    next_pose = GetPoseRequest("pre_pickup1")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)
    ur5e_arm.group.set_planner_id("LIN")


    #pre pickup1  
    next_pose = GetPoseRequest("pre_pickup1")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)

    #pickup1
    next_pose = GetPoseRequest("pickup1")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)
    gripper.close()

    #pre pickup1
    next_pose = GetPoseRequest("pre_pickup1")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)

    #pre pickup2
    next_pose = GetPoseRequest("pre_pickup2")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)

    #place2
    next_pose = GetPoseRequest("place2")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)
    gripper.open()

    #pre pickup2
    next_pose = GetPoseRequest("pre_pickup2")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)

    #pickup2
    next_pose = GetPoseRequest("pickup2")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)
    gripper.close()

    #pre pickup2
    next_pose = GetPoseRequest("pre_pickup2")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)

    #pre pickup1
    next_pose = GetPoseRequest("pre_pickup1")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)

    #place1
    next_pose = GetPoseRequest("place1")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)
    gripper.open()

    #pre pickup1  
    next_pose = GetPoseRequest("pre_pickup1")
    next_pose = get_pose_srv(next_pose)
    ur5e_arm.set_joint_value(next_pose.result)

    rospy.spin()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)
    except rospy.ROSInterruptException:
        pass