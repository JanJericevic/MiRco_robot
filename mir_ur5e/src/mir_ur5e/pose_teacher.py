#!/usr/bin/env python3

import rospy
import rospkg
import threading
import yaml

from mir_ur5e.srv import SavePose, GetPose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PoseTeacher(object):
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

        # provide a service to save a pose
        self.savePoseServer = rospy.Service("~save_arm_pose", SavePose, self.saveArmPose)

        # provide a service to drive to a pose
        self.driveToPoseServer = rospy.Service("~get_arm_pose", GetPose, self.getArmPose)

        self.node_name = rospy.get_name()
        self.save_srv_name = self.node_name + "/save_arm_pose"
        self.get_srv_name = self.node_name + "/get_arm_pose"
        self.loginfo_blue("Save arm pose service: " + self.save_srv_name)
        self.loginfo_blue("Get saved arm pose service: " + self.get_srv_name)
        self.loginfo_blue("Pose teacher initialization done")

    def loginfo_blue(self, msg):
        rospy.loginfo('\033[94m' + msg + '\033[0m')

    def setLatestJointState(self, jointState):
        with self.jointStateLock:
            self.latestJointState = jointState

    def saveArmPose(self, request):
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

            # update values
            for joint in self.relevantJoints:
                index = self.latestJointState.name.index(joint)
                value = self.latestJointState.position[index]
                joints[joint] = value

            poses[request.name] = joints

            # save to file
            with open(self.filename, 'w') as poseFile:
                yaml.dump(poses, poseFile)

        return "Pose saved"


    def getArmPose(self, request):
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

            return msg

if __name__ == '__main__':
    rospy.init_node("pose_teacher")
    pt = PoseTeacher()
    rospy.spin()