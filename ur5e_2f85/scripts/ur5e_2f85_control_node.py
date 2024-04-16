#! /usr/bin/env python

import rospy
import rospkg
from ur5e_2f85.ur5e_class import UR5e
from ur5e_2f85.robotiq_2f85_class import Robotiq2f85

def main():
    # init node
    rospy.init_node('ur5e_2f85_control_node', anonymous=True)

    # init robot arm
    rospack = rospkg.RosPack()
    package = "mirco_robot"
    file_name = rospack.get_path(package) + "/config/ur5e_saved_poses.yml" 
    ur5e_arm = UR5e("robot_arm", pose_file = file_name)

    # go home
    ur5e_arm.set_named_pose("home")

    # init robot gripper
    gripper = Robotiq2f85(max_gap=0.062)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)
    except rospy.ROSInterruptException:
        pass