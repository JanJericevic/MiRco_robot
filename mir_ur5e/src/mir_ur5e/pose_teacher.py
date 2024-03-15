#!/usr/bin/env python3

import rospy
import rospkg
import threading
import yaml

from mir_ur5e.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PoseTeacher(object):
    """Save and get robot poses as joint states
    """
    def __init__(self):
        self.loginfo_blue("Initializing pose teacher")

        # get a filename to save the poses to
        rospack = rospkg.RosPack()
        package = "mir_ur5e"
        self.filename = rospack.get_path(package) + "/config/ur5e_saved_poses.yaml"

        # subscribe to joint states
        self.latestJointState = None
        self.jointStateLock = threading.Lock()

        # get robot namespace and prefixes
        if rospy.has_param("/robot_namespace_prefix"):
            self.namespace = rospy.get_param("/robot_namespace_prefix")
        else:
            self.namespace = ""
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
        self.jointListener = rospy.Subscriber("joint_states", JointState, self.setLatestJointState, queue_size=5)

        # provide a service to save a pose as joint states
        self.savePoseServer = rospy.Service("~save_arm_pose", SavePose, self.saveArmPose)

        # provide a service to get a pose as joint states
        self.driveToPoseServer = rospy.Service("~get_arm_pose", GetPose, self.getArmPose)
        
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

    def setLatestJointState(self, jointState:JointState) -> None:
        """Update robot joint states

        :param jointState: latest joints states
        :type jointState: JointState
        """
        with self.jointStateLock:
            self.latestJointState = jointState

    def saveArmPose(self, request:SavePoseRequest) -> str:
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
                with open(self.filename) as poseFile:
                    poses = yaml.load(poseFile, Loader=yaml.SafeLoader)
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
            with open(self.filename, 'w') as poseFile:
                yaml.dump(poses, poseFile)

        self.loginfo_blue("Pose saved as: '" + request.name + "'")
        return "Pose saved"


    def getArmPose(self, request:GetPoseRequest) -> JointState:
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
                with open(self.filename) as poseFile:
                    poses = yaml.load(poseFile, Loader=yaml.SafeLoader)
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

if __name__ == '__main__':
    rospy.init_node("pose_teacher")
    pt = PoseTeacher()
    rospy.spin()