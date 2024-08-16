#! /usr/bin/env python

# Copyright 2024, Jan Jerićević

# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions
# are met:

#     * Redistributions of source code must retain the above copyright 
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above 
#     copyright notice, this list of conditions and the following 
#     disclaimer in the documentation and/or other materials provided 
#     with the distribution.

#     * Neither the name of the copyright holder nor the names of its 
#     contributors may be used to endorse or promote products derived 
#     from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.

# Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib

from ur5e_2f85.pose_teacher import PoseTeacher
from ur5e_2f85.srv import *
from sensor_msgs.msg import JointState

class UR5e:
    """Simple ur5e robot arm class. Provides moveit functionalities and custom services
    """
    def __init__(self, group_name:str, pipeline="pilz_industrial_motion_planner", planner="LIN", pose_file: str = None):
        self.loginfo_cyan("Initializing UR5e robot python commander")
        
        # get robot prefix
        if rospy.has_param("/robot_arm_prefix"):
            self.arm_prefix = rospy.get_param("/robot_arm_prefix")
        else:
            self.arm_prefix = ""

        # moveit_commander and rospy node
        self._commander = moveit_commander.roscpp_initialize(sys.argv)

        # RobotCommander 
        self.robot = moveit_commander.RobotCommander()
        # PlanningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # movegroup
        self.planning_group = group_name
        self.group = moveit_commander.MoveGroupCommander(self.planning_group)

        self.group.set_pose_reference_frame(self.arm_prefix + 'base_link_inertia')
        self.group.clear_pose_targets()

        self.group.set_max_velocity_scaling_factor(0.4)
        self.group.set_max_acceleration_scaling_factor(0.4)

        # set default planning pipeline and planner
        self.planning_pipeline = pipeline
        self.planner = planner
        self.group.set_planning_pipeline_id(self.planning_pipeline)
        self.group.set_planner_id(self.planner)
        
        # DisplayTrajectory publisher - RViz
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # action client for the Execute Trajectory action server
        self.execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self.execute_trajectory_client.wait_for_server()

        # set robot arm pose service
        self.set_pose_server = rospy.Service("~set_arm_pose", SetPose, self.set_pose_service)
        self.set_pose_srv_name = rospy.get_name()+ "/set_arm_pose"
        rospy.wait_for_service(self.set_pose_srv_name)
        self.loginfo_cyan("Set robot arm pose service: " + self.set_pose_srv_name)

        # initialize pose teacher
        # get a filename to save the poses
        if pose_file == None:
            self.pt = PoseTeacher()
        else:
            self.pt = PoseTeacher(pose_file)
        # wait for pose teacher services
        rospy.wait_for_service(self.pt.save_srv_name)
        rospy.wait_for_service(self.pt.get_srv_name)
        # create service clients
        self.save_pose_srv = rospy.ServiceProxy(self.pt.save_srv_name, SavePose)
        self.get_pose_srv = rospy.ServiceProxy(self.pt.get_srv_name, GetPose)
        
        # print robot and move group info
        self.get_info(print=True)

        # end of robot initialization
        self.loginfo_cyan("UR5e robot python commander initialization complete.")
    
    def loginfo_cyan(self, msg:str) -> None:
        """Helper function. Print loginfo message with light cyan text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[96m' + "UR5e: " + msg + '\033[0m')

    def get_info(self, print = False) -> dict:
        """Get robot and move group info

        :param print: print the info as loginfo, defaults to False
        :type print: bool, optional
        :return: dictionary with robot and move group info
        :rtype: dict
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
            # print the info
            self.loginfo_cyan("-----INFO-----")
            self.loginfo_cyan("Available planning groups: {}".format(self.group_names))
            self.loginfo_cyan("Selected planning group: {}".format(self.group_name))
            self.loginfo_cyan("Planning Frame: {}".format(self.planning_frame))
            self.loginfo_cyan("Planning pipeline: {}".format(self.planning_pipeline))
            self.loginfo_cyan("Planner: {}".format(self.planner))
            self.loginfo_cyan("Goal tolerances [joint, position, orientation]: {}".format(self.goal_tolerances))
            self.loginfo_cyan("End Effector Link: {}".format(self.eef_link))
            self.loginfo_cyan("Current joint values: {}".format(self.current_joint_values))
            self.loginfo_cyan("Pose reference frame: {}".format(self.pose_reference_frame))
            # self.loginfo_cyan("Current pose: {}".format(self.current_pose))
            self.loginfo_cyan("--------------")

        # return a dict with info
        info = {"group_names": self.group_names, "group_name": self.group_name, "planning_frame": self.planning_frame,"goal_tolerances": self.goal_tolerances , "eef_link": self.eef_link, "current_joint_values": self.current_joint_values, "pose_reference_frame": self.pose_reference_frame, "current_pose": self.current_pose}

        return info
    
    def teacher_save_pose(self, pose_name:str) -> str:
        """Save current robot pose to file

        :param pose_name: pose name
        :type pose_name: str
        :return: service result message
        :rtype: str
        """
        request = SavePoseRequest(pose_name)
        response = self.save_pose_srv(request)

        return response.result
    
    def teacher_get_pose(self, pose_name:str) -> JointState:
        """Get saved robot pose

        :param pose_name: requested pose name
        :type pose_name: str
        :return: robot joint states of requested pose
        :rtype: JointState
        """
        request = GetPoseRequest(pose_name)
        response = self.get_pose_srv(request)

        return response.result
    
    def set_pose_service(self, request: SetPoseRequest) -> SetPoseResponse:
        """ROS service callback function. Sets robot arm pose

        :param request: service request
        :type request: SetPoseRequest
        :return: service response
        :rtype: SetPoseResponse
        """
        # wait for get pose service
        get_pose_service_name = self.pt.get_srv_name
        rospy.wait_for_service(get_pose_service_name)

        # get robot pose
        try:
            get_pose_service = rospy.ServiceProxy(get_pose_service_name, GetPose)
            pose = get_pose_service(request.name)            

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        self.move_j(pose.result)

        return SetPoseResponse("Set robot pose: '" + request.name + "'")


    def set_pose(self, pose) -> None:
        """Set target pose

        :param pose: target robot pose
        :type pose: PoseStamped, Pose, [x, y, z, rot_x, rot_y, rot_z] or [x, y, z, qx, qy, qz, qw]
        """
        # clear all pose targets
        self.group.clear_pose_targets()
        self.loginfo_cyan("Going to pose: {}".format(pose))

        # set goal
        self.group.set_pose_target(pose)
        # plan to goal using the default planner
        plan_success, plan, planning_time, error_code = self.group.plan()
        # goal message
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # update the trajectory in the goal message
        goal.trajectory = plan
        # send the goal to the action server
        self.execute_trajectory_client.send_goal(goal)
        self.execute_trajectory_client.wait_for_result()
        # print the current pose
        self.current_pose = self.group.get_current_pose()
        self.loginfo_cyan("Now at Pose: {}".format(self.current_pose))

    def set_named_pose(self, pose_name:str) -> None:
        """Set a joint configuration by name specified in the SRDF

        :param pose_name: pose name in SRDF
        :type pose_name: str
        """
        # clear all pose targets
        self.group.clear_pose_targets()
        self.loginfo_cyan("Going to named pose: {}".format(pose_name))

        # set goal
        self.group.set_named_target(pose_name)
        # plan to goal using the default planner
        plan_success, plan, planning_time, error_code = self.group.plan()
        # goal message
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # update the trajectory in the goal message
        goal.trajectory = plan
        # send the goal to the action server
        self.execute_trajectory_client.send_goal(goal)
        self.execute_trajectory_client.wait_for_result()
        # Print the current pose
        #  self.current_pose = self.group.get_current_pose()
        #  self.loginfo_cyan("Now at Pose: {}".format(self.current_pose))
    
    def move_l(self, goal) -> None:
        """Linear move in cartesian space

        :param goal: goal
        :type goal: JointState or Pose
        """

        # planning pipeline check
        if self.planning_pipeline != "pilz_industrial_motion_planner":
            self.loginfo_cyan("Setting the planning pipeline to: 'pilz_industrial_motion_planner'")
            self.group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            self.group.planning_pipeline("pilz_industrial_motion_planner")
        # planner id check
        if self.planner != "LIN":
            self.loginfo_cyan("Setting the planner ID to: 'LIN'")
            self.group.set_planner_id("LIN")
            self.planner = "LIN"

        # clear all pose targets
        self.group.clear_pose_targets()
        # set scaling factors
        self.group.set_max_velocity_scaling_factor(0.4)
        self.group.set_max_acceleration_scaling_factor(0.2)
        # execute move
        self.loginfo_cyan("Setting joint value target")
        self.group.go(goal, wait=True) 
    
    def move_j(self, goal) -> None:
        """Joint move in joint space

        :param goal: goal
        :type goal: JointState or Pose
        """
         # planning pipeline check
        if self.planning_pipeline != "pilz_industrial_motion_planner":
            self.loginfo_cyan("Setting the planning pipeline to: 'pilz_industrial_motion_planner'")
            self.group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            self.group.planning_pipeline("pilz_industrial_motion_planner")
        # planner id check
        if self.planner != "PTP":
            self.loginfo_cyan("Setting the planner ID to: 'PTP'")
            self.group.set_planner_id("PTP")
            self.planner = "PTP"

        # clear all pose targets
        self.group.clear_pose_targets()
        # set scaling factors
        self.group.set_max_velocity_scaling_factor(0.4)
        self.group.set_max_acceleration_scaling_factor(0.2)
        # execute move
        self.loginfo_cyan("Setting joint value target")
        self.group.go(goal, wait=True)   

    #  Class Destructor
    def __del__(self):
        # when the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        self.loginfo_cyan(
            "Object of class ur5e Deleted.")


def main():
    rospy.init_node('ur5e_robot_python_node', anonymous=True)

    # create a new manipulator object
    ur5e_arm = UR5e("arm")
    ur5e_arm.get_info(print=True)

    # delete the manipulator object
    del manipulator

    rospy.signal_shutdown("UR5e robot python commander deleted")


if __name__ == '__main__':
    main()

