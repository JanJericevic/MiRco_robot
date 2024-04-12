#!/usr/bin/env python3

import rospy
import rospkg
import tf
import yaml

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
        self.save_srv_name = self.node_name + "/save_mobile_goal"
        self.get_srv_name = self.node_name + "/get_mobile_goal"
        self.get_names_srv_name = self.node_name + "/get_mobile_goal_names"
        self.loginfo_blue("Save mobile goal service: " + self.save_srv_name)
        self.loginfo_blue("Get mobile goal service: " + self.get_srv_name)
        self.loginfo_blue("Get mobile goal names service: " + self.get_names_srv_name)
        self.loginfo_blue("Goal teacher initialization done")

    def loginfo_blue(self, msg:str) -> None:
        """Helper function. Print loginfo message with light blue text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[34m' + "Goal Teacher: " + msg + '\033[0m')


    def save_target_goal(self, request:SaveGoalRequest) -> str:
        """Save current mobile robot pose as a target goal

        :param request: service request. calls SaveGoal service
        :type request: SaveGoalRequest
        :raises Exception: if no transform is received
        :return: message when finished
        :rtype: str
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

        # update goal values
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

        self.loginfo_blue("Target goal saved as: '" + request.name + "'")
        return "Target goal saved"


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