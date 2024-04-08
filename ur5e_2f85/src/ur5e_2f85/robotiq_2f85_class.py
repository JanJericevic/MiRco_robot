#!/usr/bin/env python

import time
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

class Robotiq2f85:
    """Simple robotiq 2f85 gripper class
    """

    def __init__(self, max_gap = 0.085, min_gap = 0, opened_value = 0, closed_value = 255):
        self.loginfo_yellow("Initializing 2f85 gripper python commander")
        self.gripper_status_received = False

        # gripper command publisher
        self.cmd_pub = rospy.Publisher("output", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        # gripper status subscriber
        self.status_sub = rospy.Subscriber("input", inputMsg.Robotiq2FGripper_robot_input,  self.get_status)

        # give time for subscribing
        rospy.sleep(1)

        # max and min gap
        self.max_gap = max_gap
        self.max_gap_mm = self.max_gap * 1000
        self.min_gap = min_gap
        self.min_gap_mm = self.min_gap * 1000
        # values reported by the gripper when opened/closed
        self.gripper_opened = opened_value
        self.gripper_closed = closed_value

        if self.is_connected():
            self.loginfo_yellow("Connected to gripper")
        else:
            for i in range(10):
                self.loginfo_yellow("Failed to connect to gripper. Retrying (" + str(i+1) + "/10)")
                if self.is_connected():
                    self.loginfo_yellow("Connected to gripper")
                    break
                if i == 9:
                    rospy.logerr("Failed to connect to gripper.")
                    raise Exception("Failed to connect to gripper.")
                
                rospy.sleep(1)
                i += 1


        # gripper reset
        self.loginfo_yellow("Gripper reset needed")
        self.reset()

        # calibrate gripper
        self.loginfo_yellow("Calibrating gripper")
        self.calib()

        self.loginfo_yellow("2f85 gripper python commander initialization complete")
    
    def loginfo_yellow(self, msg:str) -> None:
        """Helper function. Print loginfo message with light yellow text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[93m' + "Gripper: " + msg + '\033[0m')

    def get_status(self,msg):
        """Update gripper status

        :param msg: data from gripper input topic
        :type msg: Robotiq2FGripper_robot_input
        """
        if self.gripper_status_received == False:
            self.gripper_status_received = True
        self.gripper_status = msg
    
    def pub_cmd_msg(self):
        """Publish gripper command to gripper output topic
        """
        self.loginfo_yellow(self.info_msg)
        self.cmd_pub.publish(self.cmd_msg)
    
    # check functions 
    def is_connected(self):
        """Helper function. Check if connected to gripper

        :return: connection status
        :rtype: bool
        """
        return self.gripper_status_received

    def is_reset(self):
        """Helper function. Check if gripper is reset

        :return: reset status
        :rtype: bool
        """
        return self.gripper_status.gSTA == 0 or self.gripper_status.gACT == 0
    
    def is_ready(self):
        """Helper function. Check if gripper is ready

        :return: ready status
        :rtype: bool
        """
        return self.gripper_status.gSTA == 3 and self.gripper_status.gACT == 1
    
    def is_moving(self):
        """Helper function. Check if gripper is moving

        :return: moving status
        :rtype: bool
        """
        return self.gripper_status.gGTO == 1 and self.gripper_status.gOBJ == 0
    
    def is_stopped(self):
        """Helper function. Check if gripper has stopped

        :return: stop status
        :rtype: bool
        """
        return self.gripper_status.gOBJ != 0

    # before use the gripper has to be reset
    def reset(self):
        """Reset gripper
        """
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.info_msg = "Resetting gripper"
        # reset command
        self.cmd_msg.rACT = 0
        # send command
        self.pub_cmd_msg()
        # wait for execution
        rospy.sleep(1)
        # check if reset
        if self.is_reset():
            self.loginfo_yellow("Reset complete")
        else:
            rospy.logwarn("Reset incomplete")
    
    # after reset, the gripper has to be activated
    def activate(self):
        """Activate gripper
        """
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.info_msg = "Activating gripper"
        # activate command
        self.cmd_msg.rACT = 1
        self.cmd_msg.rGTO = 1
        self.cmd_msg.rSP = 255
        self.cmd_msg.rFR = 150
        # send activate command
        self.pub_cmd_msg()
        # wait for execution
        rospy.sleep(2)
        # check if ready
        if self.is_ready():
            self.loginfo_yellow("Activation complete")
        else:
            rospy.logwarn("Activation incomplete")
    
    def close(self):
        """Close gripper
        """
        self.info_msg = "Closing gripper"
        # close command
        self.cmd_msg.rPR = 255
        # send command
        self.pub_cmd_msg()
        # wait for execution
        rospy.sleep(1)

    def open(self):
        """Open gripper
        """
        self.info_msg = "Opening gripper"
        # open command
        self.cmd_msg.rPR = 0
        # send command
        self.pub_cmd_msg()
        # wait for execution
        rospy.sleep(1)
    
    def calib(self):
        """Calibrate gripper. Sets gripper open/closed values
        """
        # activate gripper
        self.activate()
        # read real open gripper value
        self.gripper_opened = self.gripper_status.gPO
        self.info_msg = "Calibrating open gripper value: " + str(self.gripper_opened)
        self.loginfo_yellow(self.info_msg)
        # close gripper
        self.close()
        # read real closed gripper value
        self.gripper_closed = self.gripper_status.gPO
        self.info_msg = "Calibrating closed gripper value: " + str(self.gripper_closed)
        self.loginfo_yellow(self.info_msg)
        # open gripper
        self.open()
        # calibrate gripper range
        self.gripper_range = self.gripper_closed - self.gripper_opened
        self.tick_per_dist = self.gripper_range / self.max_gap_mm

        self.loginfo_yellow("Calibration complete")
    
    #ACCURACY CAN BE IMPROVED
    def set_gap_mm(self,mm):
        """Close gripper to gap set in mm. Before use have to use the calib() method

        :param mm: gap in mm
        :type mm: int
        """
        self.info_msg = "Closing gripper to gap:" + str(mm) + "mm"
        rospy.loginfo(self.info_msg)
        # calculate int value to send to gripper from desired gripper gap in mm
        position = self.tick_per_dist * (self.max_gap_mm - mm)
        position = round(position)

        self.set_gap_position(position)
    
    # close gripper to gap in values from self.gripper_opened - self.gripper_closed
    def set_gap_position(self,position):
        """Set gripper gap position in values from 0-255. 0: fully open, 255: fully closed

        :param position: position
        :type position: int
        """
        self.info_msg = "Sending gripper to position:" + str(position)
        # position command
        self.cmd_msg.rPR = int(position)
        if self.cmd_msg.rPR > 255:
            self.cmd_msg.rPR = 255
        if self.cmd_msg.rPR < 0:
            self.cmd_msg.rPR = 0
        # send command
        self.pub_cmd_msg()
        # wait for execution
        rospy.sleep(2)

    def inc_speed(self):
        """Increase gripper speed
        """
        self.info_msg = "Increasing gripper speed"
        self.cmd_msg.rSP += 25
        if self.cmd_msg.rSP > 255:
            self.cmd_msg.rSP = 255
        
        self.pub_cmd_msg()
    
    def dec_speed(self):
        """Decrease gripper speed
        """
        self.info_msg = "Decreasing gripper speed"
        self.cmd_msg.rSP -= 25
        if self.cmd_msg.rSP < 0:
            self.cmd_msg.rSP = 0
    
        self.pub_cmd_msg()
    
    def inc_force(self):
        """Increase gripper force
        """
        self.info_msg = "Increasing gripper force"
        self.cmd_msg.rFR += 25
        if cself.md_msg.rFR > 255:
            self.cmd_msg.rFR = 255
    
        self.pub_cmd_msg()

    def dec_force(self):
        """Decrease gripper force
        """
        self.info_msg = "Decreasing gripper force"
        self.cmd_msg.rFR -= 25
        if self.cmd_msg.rFR < 0:
            self.cmd_msg.rFR = 0
    
        self.pub_cmd_msg()