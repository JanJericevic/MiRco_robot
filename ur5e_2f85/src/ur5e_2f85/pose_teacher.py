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
import threading
import yaml

from ur5e_2f85.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PoseTeacher(object):
    """Save and get robot poses as joint states
    """
    def __init__(self,pose_file: str = None):
        self.loginfo_blue("Initializing pose teacher")

        # get a filename to save the poses
        if pose_file == None:
            rospack = rospkg.RosPack()
            package = "ur5e_2f85"
            self.filename = rospack.get_path(package) + "/config/ur5e_saved_poses.yml"
        else:
            self.filename = pose_file

        # subscribe to joint states
        self.latestJointState = None
        self.jointStateLock = threading.Lock()

        # get robot prefix
        if rospy.has_param("/robot_arm_prefix"):
            self.arm_prefix = rospy.get_param("/robot_arm_prefix")
        else:
            self.arm_prefix = ""

        # joint names
        self.relevantJoints = [
            self.arm_prefix + 'shoulder_pan_joint',
            self.arm_prefix + 'shoulder_lift_joint',
            self.arm_prefix + 'elbow_joint',
            self.arm_prefix + 'wrist_1_joint',
            self.arm_prefix + 'wrist_2_joint',
            self.arm_prefix + 'wrist_3_joint',
        ]

        # joint state subscriber
        self.jointListener = rospy.Subscriber("joint_states", JointState, self.set_latest_joint_state, queue_size=5)

        # provide a service to save a pose as joint states
        self.save_pose_server = rospy.Service("~save_arm_pose", SavePose, self.save_arm_pose)

        # provide a service to get a pose as joint states
        self.get_pose_server = rospy.Service("~get_arm_pose", GetPose, self.get_arm_pose)

        # provide a service to get a list of saved poses
        self.get_pose_names_server = rospy.Service("~get_arm_pose_names", GetPoseNames, self.get_arm_pose_names)
        
        # print service addresses
        self.node_name = rospy.get_name()
        self.save_srv_name = self.node_name + "/save_arm_pose"
        self.get_srv_name = self.node_name + "/get_arm_pose"
        self.loginfo_blue("Save arm pose service: " + self.save_srv_name)
        self.loginfo_blue("Get saved arm pose service: " + self.get_srv_name)
        self.loginfo_blue("Pose teacher initialization done")

    def loginfo_blue(self, msg:str) -> None:
        """Helper function. Print loginfo message with light blue text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[94m' + "Pose Teacher: " + msg + '\033[0m')

    def set_latest_joint_state(self, jointState:JointState) -> None:
        """Update robot joint states

        :param jointState: latest joints states
        :type jointState: JointState
        """
        with self.jointStateLock:
            self.latestJointState = jointState

    def save_arm_pose(self, request:SavePoseRequest) -> str:
        """Save the robot pose as joint states

        :param request: service request. calls SavePose service
        :type request: SavePoseRequest
        :raises Exception: if no JointState is received
        :return: message when finished
        :rtype: str
        """
        with self.jointStateLock:
            if self.latestJointState is None:
                raise Exception("No JointState received yet")

            # read from file
            try:
                with open(self.filename) as pose_file:
                    poses = yaml.load(pose_file, Loader=yaml.SafeLoader)
                    if poses == None:
                        poses = {}
            except IOError as err:
                print("Could not open %s to read poses" % self.filename)
                print(err)

            joints = {}
            if request.name in poses:
                joints = poses[request.name]

            # update joint values
            for joint in self.relevantJoints:
                index = self.latestJointState.name.index(joint)
                value = self.latestJointState.position[index]
                joints[joint] = value

            poses[request.name] = joints

            # save to file
            with open(self.filename, 'w') as pose_file:
                yaml.dump(poses, pose_file)

        self.loginfo_blue("Pose saved as: '" + request.name + "'")
        return "Pose saved"


    def get_arm_pose(self, request:GetPoseRequest) -> JointState:
        """Get a saved robot pose as joint states

        :param request: service request. calls GetPose service
        :type request: GetPoseRequest
        :raises Exception: if saved poses list is empty
        :return: saved joint states
        :rtype: JointState
        """
        with self.jointStateLock:
            # read the config file
            try:
                with open(self.filename) as pose_file:
                    poses = yaml.load(pose_file, Loader=yaml.SafeLoader)
                    if poses == None:
                        rospy.logwarn("Saved poses list is empty")
            except IOError as err:
                print("Could not open %s to read poses" % self.filename)
                print(err)
                raise

            if request.name not in poses:
                raise Exception("Unknown pose: %s" % request.name)

            # create a JointState message
            msg = JointState()
            for key, value in poses[request.name].items():
                msg.name.append(key)
                msg.position.append(value)

            self.loginfo_blue("Returned pose: '" + request.name + "'")
            return msg
        
    def get_arm_pose_names(self, request:GetPoseNamesRequest) -> list:
        """Get a list of saved poses

        :param request: service request. calls GetPoseNames service
        :type request: GetPoseNamesRequest
        :raises Exception: if saved poses list is empty
        :return: saved poses names
        :rtype: list
        """
        with self.jointStateLock:
            # read the config file
            try:
                with open(self.filename) as pose_file:
                    poses = yaml.load(pose_file, Loader=yaml.SafeLoader)
                    if poses == None:
                        rospy.logwarn("Saved poses list is empty")
            except IOError as err:
                print("Could not open %s to read poses" % self.filename)
                print(err)
                raise

            pose_names = []
            for key in poses.keys():
                pose_names.append(key)

            self.loginfo_blue("Returned saved pose names")
            return [pose_names]

if __name__ == '__main__':
    rospy.init_node("pose_teacher")
    pt = PoseTeacher()
    rospy.spin()