#! /usr/bin/env python

#Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

class UR5e:
    """simple ur5e robot arm class
    """

    def __init__(self, group_name):
        #get robot namespace and prefixes
        self.namespace = rospy.get_param("/robot_namespace_prefix")
        self.arm_prefix = rospy.get_param("/robot_arm_prefix")

        #moveit_commander and rospy node
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5e_robot_python_node', anonymous=True)

        #RobotCommander 
        self.robot = moveit_commander.RobotCommander()
        #PlanningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()
        
        #movegroup
        self.planning_group = group_name
        self.group = moveit_commander.MoveGroupCommander(self.planning_group)

        self.group.set_pose_reference_frame('base_link_inertia')
        self.group.clear_pose_targets()

        self.group.set_max_velocity_scaling_factor(0.4)
        self.group.set_max_acceleration_scaling_factor(0.4)
        
        #DisplayTrajectory publisher - RViz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        #action client for the Execute Trajectory action server
        self.execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self.execute_trajectory_client.wait_for_server()

        rospy.loginfo('\033[95m' + " >>> ur5e moveit script initialization is done." + '\033[0m')

    def get_info(self, print = False):
        """get robot and move group info 

        :return: _description_
        :rtype: _type_
        """
        self.group_names = self.robot.get_group_names()
        self.group_name = self.group.get_name()
        self.planning_frame = self.group.get_planning_frame()
        self.goal_tolerances = self.group.get_goal_tolerance()
        self.eef_link = self.group.get_end_effector_link()
        self.current_joint_values = self.group.get_current_joint_values()
        self.pose_reference_frame = self.group.get_pose_reference_frame()
        self.current_pose = self.group.get_current_pose()

        if print:
            #print the info
            rospy.loginfo('\033[95m' + "Available planning groups: {}".format(self.group_names) + '\033[0m')
            rospy.loginfo('\033[95m' + "Selected planning group: {}".format(self.group_name) + '\033[0m')
            rospy.loginfo('\033[95m' + "Planning Frame: {}".format(self.planning_frame) + '\033[0m')
            rospy.loginfo('\033[95m' + "Goal tolerances [joint, position, orientation]: {}".format(self.goal_tolerances) + '\033[0m')
            rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self.eef_link) + '\033[0m')
            rospy.loginfo('\033[95m' + "Current joint values: {}".format(self.current_joint_values) + '\033[0m')
            rospy.loginfo('\033[95m' + "Pose reference frame: {}".format(self.pose_reference_frame) + '\033[0m')
            rospy.loginfo('\033[95m' + "Current pose: {}".format(self.current_pose) + '\033[0m')

            #'\033[95m' is the color "LightMagenta" (https://pkg.go.dev/github.com/whitedevops/colors)
            #'\033[0m' is the default color

        #return a dict with info
        info = {"group_names": self.group_names, "group_name": self.group_name, "planning_frame": self.planning_frame,"goal_tolerances": self.goal_tolerances , "eef_link": self.eef_link, "current_joint_values": self.current_joint_values, "pose_reference_frame": self.pose_reference_frame, "current_pose": self.current_pose}

        return info

    def set_pose(self, pose):
        """set pose as PoseStamped, Pose, [x, y, z, rot_x, rot_y, rot_z] or [x, y, z, qx, qy, qz, qw]

        :param pose: _description_
        :type pose: _type_
        """
        self.group.clear_pose_targets()
        
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(pose) + '\033[0m')

        #set goal
        self.group.set_pose_target(pose)
        #plan to goal using the default planner
        plan_success, plan, planning_time, error_code = self.group.plan()
        #goal message
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        #update the trajectory in the goal message
        goal.trajectory = plan
        #send the goal to the action server
        self.execute_trajectory_client.send_goal(goal)
        self.execute_trajectory_client.wait_for_result()
        #Print the current pose
        self.current_pose = self.group.get_current_pose()
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(self.current_pose) + '\033[0m')

    def set_named_pose(self, pose_name):
        """Set a joint configuration by name specified in the SRDF

        :param pose_name: _description_
        :type pose_name: _type_
        """
        self.group.clear_pose_targets()
        # self.group.set_max_velocity_scaling_factor(0.1)
        # self.group.set_max_acceleration_scaling_factor(0.1)
        
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(pose_name) + '\033[0m')

        #set goal
        self.group.set_named_target(pose_name)
        #plan to goal using the default planner
        plan_success, plan, planning_time, error_code = self.group.plan()
        #goal message
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        #update the trajectory in the goal message
        goal.trajectory = plan
        #send the goal to the action server
        self.execute_trajectory_client.send_goal(goal)
        self.execute_trajectory_client.wait_for_result()
        #Print the current pose
        # self.current_pose = self.group.get_current_pose()
        # rospy.loginfo('\033[32m' + "Now at Pose: {}".format(self.current_pose) + '\033[0m')

    
    def moveJ(self, pose_name):
        self.group.clear_pose_targets()

        # self.group.set_planner_id('PTP')
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
        
        # plan and execute
        self.group.set_named_target(pose_name)
        self.group.go(wait=True)  
        self.group.stop()

    def pick_and_place(self):
        self.set_pose("pickup")
        self.set_pose("pitstop")
        self.set_pose("drop")

        rospy.sleep(1)

        self.set_pose("pitstop")
        self.set_pose("pickup")

    # Class Destructor
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class ur5e Deleted." + '\033[0m')


def main():

    #Create a new manipulator object
    ur5e_arm = UR5e("arm")
    manipulator.pick_and_place()

    #delete the manipulator object
    del manipulator

    rospy.signal_shutdown("motion concluded")


if __name__ == '__main__':
    main()

