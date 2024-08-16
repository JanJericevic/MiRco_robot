#!/usr/bin/env python

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

""" ROS joystick teleoperation script. Optional REST API interaction for MiR 100 robot. 

Requirements:
To get joystick data, ROS joy_node is needed (http://wiki.ros.org/joy). 
To interact with the robot's REST API, the script uses imported MirRestApi class from the mir_rest_api ROS package.

Instructions:
Customize button configuration accordingly.
Used button configuration for Logitech F710 joystick.

left stick vertical axis: linear velocity
right stick horizontal axis: angular velocity

left bumper: increase stick linear velocity range
left trigger: decrease stick linear velocity range
right bumper: increase stick angular velocity range
right trigger: decrease stick angular velocity range

MiR 100 REST API interactions:
x: toggle robot state - Ready/Pause. Used to start/stop execution of mission queue
a: retrieve current robot mode
b: delete mission queue
y: retrieve current robot status
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mir_rest_api.api import MirRestApi
from pprint import pprint

class JoyTeleop:
    """Joystick teleop class

    :param api: class used for MiR 100 REST API communication, defaults to None
    :type api: MirRestApi, optional
    """

    def __init__(self, api: MirRestApi=None) -> None:
        self.vel = Twist()
        self.range_lin_vel = 0.2
        self.range_ang_vel = 0.2
        self.min_lin_vel = 0.1
        self.min_ang_vel = 0.1
        self.max_lin_vel = 2.0
        self.max_ang_vel = 1.0

        self.api = api

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub = rospy.Subscriber("joy", Joy, self.read_joy)

    def read_joy(self, data: Joy) -> None:
        """Handles joystick data

        :param data: ROS joystick data
        :type data: Joy
        """
        self.vel.linear.x = self.range_lin_vel*data.axes[1]
        # self.vel.angular.z = self.range_ang_vel*data.axes[2]

        # ps3 controlles angular velocity
        self.vel.angular.z = self.range_ang_vel*data.axes[2]
       
       # adjust velocity ranges
       # only when robot is stationary
        if self.vel.linear.x == 0 and self.vel.angular.z == 0:
            # TODO: simultaneous stick and button data bug
            if (data.buttons[4] == 1) and (self.range_lin_vel < self.max_lin_vel):
                self.range_lin_vel = round(self.range_lin_vel + 0.1,  1)
                rospy.loginfo("MAX LIN VELOCITY = %s m/s", self.range_lin_vel)

            elif (data.buttons[6] == 1) and (self.range_lin_vel > self.min_lin_vel):
                self.range_lin_vel = round(self.range_lin_vel - 0.1, 1)
                rospy.loginfo("MAX LIN VELOCITY = %s m/s", self.range_lin_vel)

            elif (data.buttons[5] == 1) and (self.range_ang_vel < self.max_ang_vel):
                self.range_ang_vel = round(self.range_ang_vel + 0.1, 1)
                rospy.loginfo("MAX ANG VELOCITY = %s rad/s", self.range_ang_vel)

            elif (data.buttons[7] == 1) and (self.range_ang_vel > self.min_ang_vel):
                self.range_ang_vel = round(self.range_ang_vel - 0.1, 1)
                rospy.loginfo("MAX ANG VELOCITY = %s rad/s", self.range_ang_vel)

        # using joystick & MiR100 robot REST API
        if data.buttons[0] == 1:
            # x - blue button
            response = self.api.status_state_id_toggle_put(False)
            pprint(response)
        if data.buttons[1] == 1:
            # A - green button
            response = self.api.status_mode_get(False)
            pprint(response)
        if data.buttons[2] == 1:
            # B - red button
            response = self.api.mission_queue_delete(False)
            pprint(response)
        if data.buttons[3] == 1:
            # Y - yellow button
            response = self.api.status_get(False)
            pprint(response)
        
    def publish(self):
        """Publishes velocity values to 'cmd_vel' topic
        """
        self.pub.publish(self.vel)

def main(): 

    print("""
    Set your MiR interface User Name and Password.
    Set robot IP
    """)

    # get the MiR100 ip
    mir_ip = rospy.get_param("/mir_ip")

    # if using ROS service for REST requests
    # rospy.wait_for_service('mir_rest_api_service')

    api = MirRestApi("UserName", "Password", mir_ip)  
    
    rospy.init_node('joy_teleop_node', anonymous=True)
    jt = JoyTeleop(api)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        jt.publish()
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass