#!/usr/bin/env python3

import rospy
import rospkg
import tf
import yaml

from mir_control.srv import *
from geometry_msgs.msg import Pose

class GoalTeacher(object):
    """Save and get mobile robot goals
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

        # provide a service to get a target goal as a PoseStamped msg
        self.get_goal_server = rospy.Service("~get_mobile_goal", GetGoal, self.get_target_goal)
        
        # print service addresses
        self.node_name = rospy.get_name()
        self.save_srv_name = self.node_name + "/save_mobile_goal"
        self.get_srv_name = self.node_name + "/get_mobile_goal"
        self.loginfo_blue("Save mobile goal service: " + self.save_srv_name)
        self.loginfo_blue("Get mobile goal service: " + self.get_srv_name)
        self.loginfo_blue("Goal teacher initialization done")

    def loginfo_blue(self, msg:str) -> None:
        """Helper function. Print loginfo message with light blue text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[34m' + "Goal Teacher: " + msg + '\033[0m')


    def save_target_goal(self, request:SaveGoalRequest) -> str:
        """Save current mobile robot pose

        :param request: service request. calls SaveGoal service
        :type request: SaveGoalRequest
        :raises Exception: if no PoseStamped is received
        :return: message when finished
        :rtype: str
        """

        # get transform 
        try:
            self.tf_listener.waitForTransform("/map", self.mobile_frame, rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform('/map', self.mobile_frame, rospy.Time(0))
            test = (trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            raise Exception("/map to " + self.mobile_frame + " transform not received")
        
        # read from file
        # try:
        #     with open(self.filename) as goal_file:
        #         goals = yaml.load(goal_file, Loader=yaml.SafeLoader)
        #         if goals == None:
        #             goals = {}
        # except IOError as err:
        #     print("Could not open %s to read target goals" % self.filename)
        #     print(err)


        # save to file
        with open(self.filename, 'w') as goal_file:
            yaml.dump(test, goal_file)

        self.loginfo_blue("Target goal saved as: '" + request.name + "'")
        return "Target goal saved"


    # def get_target_goal(self, request:GetPoseRequest) -> JointState:
    #     """Get a saved robot pose as joint states

    #     :param request: service request. calls GetPose service
    #     :type request: GetPoseRequest
    #     :raises Exception: if saved poses list is empty
    #     :return: saved joint states
    #     :rtype: JointState
    #     """
    #     with self.jointStateLock:
    #         # read the config file
    #         try:
    #             with open(self.filename) as poseFile:
    #                 poses = yaml.load(poseFile, Loader=yaml.SafeLoader)
    #                 if poses == None:
    #                     rospy.logwarn("Saved poses list is empty")
    #         except IOError as err:
    #             print("Could not open %s to read poses" % self.filename)
    #             print(err)
    #             raise

    #         if request.name not in poses:
    #             raise Exception("Unknown pose: %s" % request.name)

    #         # create a JointState message
    #         msg = JointState()
    #         for key, value in poses[request.name].items():
    #             msg.name.append(key)
    #             msg.position.append(value)

    #         self.loginfo_blue("Returned pose: '" + request.name + "'")
    #         return msg

if __name__ == '__main__':
    rospy.init_node("goal_teacher")
    gt = GoalTeacher()
    rospy.spin()