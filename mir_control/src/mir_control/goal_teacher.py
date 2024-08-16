#!/usr/bin/env python3

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

import rospy
import rospkg
import tf
import yaml
import math

from mir_control.srv import *
from geometry_msgs.msg import Pose

class GoalTeacher(object):
    """Save and get mobile robot goals to/from file
    """
    def __init__(self):
        self.loginfo_blue("Initializing goal teacher")

        # get a filename to save the target goals
        rospack = rospkg.RosPack()
        package = "mir_control"
        self.filename = rospack.get_path(package) + "/config/target_goals.yaml"

        # get robot prefix
        if rospy.has_param("/robot_base_prefix"):
            self.base_prefix = rospy.get_param("/robot_base_prefix")
        else:
            self.base_prefix = ""

        # create tf listener object and wait for transform
        self.mobile_frame = self.base_prefix + "base_link"
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("/map", self.mobile_frame, rospy.Time(), rospy.Duration(4.0))

        # provide a service to save a target goal
        self.save_goal_server = rospy.Service("~save_mobile_goal", SaveGoal, self.save_target_goal)

        # provide a service to get a target goal
        self.get_goal_server = rospy.Service("~get_mobile_goal", GetGoal, self.get_target_goal)

        # provide a service to get a list of saved goals
        self.get_names_server = rospy.Service("~get_mobile_goal_names", GetGoalNames, self.get_target_goal_names)
        
        # print service addresses
        self.node_name = rospy.get_name()
        # self.save_srv_name = self.node_name + "/save_mobile_goal"
        self.get_srv_name = self.node_name + "/get_mobile_goal"
        self.get_names_srv_name = self.node_name + "/get_mobile_goal_names"
        # self.loginfo_blue("Save mobile goal service: " + self.save_srv_name)
        self.loginfo_blue("Get mobile goal service: " + self.get_srv_name)
        self.loginfo_blue("Get mobile goal names service: " + self.get_names_srv_name)
        self.loginfo_blue("Goal teacher initialization done")

    def loginfo_blue(self, msg:str) -> None:
        """Helper function. Print loginfo message with light blue text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[34m' + "Goal Teacher: " + msg + '\033[0m')

    def save_target_goal(self, request:SaveGoalRequest) -> list:
        """Save current mobile robot pose as a target goal

        :param request: service request. calls SaveGoal service
        :type request: SaveGoalRequest
        :raises Exception: if no transform is received
        :return: [saved goal, ROS service response]
        :rtype: list
        """

        # get transform 
        try:
            self.tf_listener.waitForTransform("/map", self.mobile_frame, rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = self.tf_listener.lookupTransform('/map', self.mobile_frame, rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            raise Exception("/map to " + self.mobile_frame + " transform not received")
        
        # read from file
        try:
            with open(self.filename) as goal_file:
                saved_goals = yaml.load(goal_file, Loader=yaml.SafeLoader)
                if saved_goals == None:
                    saved_goals = {}
        except IOError as err:
            print("Could not open %s to read target goals" % self.filename)
            print(err)

        # check if goal already exists
        goal = {}
        if request.name in saved_goals:
            goal = saved_goals[request.name]
            self.loginfo_blue("Goal with name: '" + request.name +  "' already exists. Overwriting...")

        # update goal values in file
        position={}
        position["x"] = trans[0]
        position["y"] = trans[1]
        position["z"] = trans[2]
        goal['position'] = position

        orientation={}
        orientation["x"] = rot[0]
        orientation["y"] = rot[1]
        orientation["z"] = rot[2]
        orientation["w"] = rot[3]
        goal['orientation'] = orientation
     
        saved_goals[request.name] = goal

        # save to file
        with open(self.filename, 'w') as goal_file:
            yaml.dump(saved_goals, goal_file)            

        self.loginfo_blue("Target goal saved to file as: '" + request.name + "'")
        msg = "Target goal saved to file"
        return [goal, msg]

    def get_target_goal(self, request:GetGoalRequest) -> Pose:
        """Get a saved target goal as a Pose msg

        :param request: service request. calls GetPose service
        :type request: GetPoseRequest
        :raises Exception: if requested goal does not exist
        :return: target goal as a Pose msg
        :rtype: Pose
        """
        # read the config file
        try:
            with open(self.filename) as goal_file:
                target_goals = yaml.load(goal_file, Loader=yaml.SafeLoader)
                if target_goals == None:
                    rospy.logwarn("Saved goals list is empty")
        except IOError as err:
            print("Could not open %s to read target goals" % self.filename)
            print(err)
            raise

        if request.name not in target_goals:
            raise Exception("Unknown target goal: %s" % request.name)

        goal = target_goals[request.name]
        # create a Pose message
        msg = Pose()
        msg.position.x = goal['position']['x']
        msg.position.y = goal['position']['y']
        msg.position.z = goal['position']['z']

        msg.orientation.x = goal['orientation']['x']
        msg.orientation.y = goal['orientation']['y']
        msg.orientation.z = goal['orientation']['z']
        msg.orientation.w = goal['orientation']['w']
            
        self.loginfo_blue("Returned target goal: '" + request.name + "'")
        return msg

    def get_target_goal_names(self, request:GetGoalNamesRequest) -> list:
        """Get a list of saved target goal names

        :param request: service request. calls GetGoalNames service
        :type request: GetGoalNamesRequest
        :return: saved target goal names
        :rtype: list
        """
        # read the config file
        try:
            with open(self.filename) as goal_file:
                target_goals = yaml.load(goal_file, Loader=yaml.SafeLoader)
                if target_goals == None:
                    rospy.logwarn("Saved goals list is empty")
        except IOError as err:
            print("Could not open %s to read target goals" % self.filename)
            print(err)
            raise
        
        target_goal_names = []
        for key in target_goals.keys():
            target_goal_names.append(key)
        
        self.loginfo_blue("Returned target goal names")
        return [target_goal_names]


if __name__ == '__main__':
    rospy.init_node("goal_teacher")
    gt = GoalTeacher()
    rospy.spin()