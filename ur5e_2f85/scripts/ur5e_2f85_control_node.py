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