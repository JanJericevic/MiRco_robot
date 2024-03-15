#!/usr/bin/env python

import time
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

class Robotiq2f85:
    """simple robotiq 2f85 gripper class
    """

    def __init__(self, max_gap = 0.085, min_gap = 0, opened_value = 0, closed_value = 255):
        self.gripper_status_received = False

        #gripper command publisher
        self.cmd_pub = rospy.Publisher("output", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        #gripper status subscriber
        self.status_sub = rospy.Subscriber("input", inputMsg.Robotiq2FGripper_robot_input,  self.get_status)

        #max and min gap
        self.max_gap = max_gap
        self.max_gap_mm = self.max_gap * 1000
        self.min_gap = min_gap
        self.min_gap_mm = self.min_gap * 1000
        #values reported by the gripper when opened/closed
        self.gripper_opened = opened_value
        self.gripper_closed = closed_value

        rospy.sleep(3)

        if self.is_connected():
            self.loginfo_magenta("Connected to gripper")
        else:
            return False

        #gripper info msg
        self.info_msg = "Gripper reset needed"
        self.loginfo_magenta(self.info_msg)

        self.reset()
        rospy.sleep(1)
        self.loginfo_magenta("Calibrating gripper")
        self.calib()
    
    def loginfo_magenta(self, msg):
        rospy.loginfo('\033[95m' + msg + '\033[0m')

    def get_status(self,msg):
        if self.gripper_status_received == False:
            self.gripper_status_received = True
        self.gripper_status = msg
    
    def pub_cmd_msg(self):
        self.loginfo_magenta(self.info_msg)
        self.cmd_pub.publish(self.cmd_msg)
    
    #check functions 
    def is_connected(self):
        return self.gripper_status_received

    def is_reset(self):
        return self.gripper_status.gSTA == 0 or self.gripper_status.gACT == 0
    
    def is_ready(self):
        return self.gripper_status.gSTA == 3 and self.gripper_status.gACT == 1
    
    def is_moving(self):
        return self.gripper_status.gGTO == 1 and self.gripper_status.gOBJ == 0
    
    def is_stopped(self):
        return self.gripper_status.gOBJ != 0

    #before use the gripper has to be reset
    def reset(self):
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.info_msg = "Resetting gripper"
        #reset command
        self.cmd_msg.rACT = 0
        #send command
        self.pub_cmd_msg()
        #wait for execution
        rospy.sleep(1)
        #check if reset
        if self.is_reset():
            self.loginfo_magenta("Reset complete")
        else:
            rospy.logwarn("Reset incomplete")
    
    #after reset, the gripper has to be activated
    def activate(self):
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.info_msg = "Activating gripper"
        #activate command
        self.cmd_msg.rACT = 1
        self.cmd_msg.rGTO = 1
        self.cmd_msg.rSP = 255
        self.cmd_msg.rFR = 150
        #send activate command
        self.pub_cmd_msg()
        #wait for execution
        rospy.sleep(3)
        #check if ready
        if self.is_ready():
            self.loginfo_magenta("Activation complete")
        else:
            rospy.logwarn("Activation incomplete")
    
    def close(self):
        self.info_msg = "Closing gripper"
        #close command
        self.cmd_msg.rPR = 255
        #send command
        self.pub_cmd_msg()
        #wait for execution
        rospy.sleep(2)

    def open(self):
        self.info_msg = "Opening gripper"
        #open command
        self.cmd_msg.rPR = 0
        #send command
        self.pub_cmd_msg()
        #wait for execution
        rospy.sleep(2)
    
    def calib(self):
        #activate gripper
        self.activate()
        #read real open gripper value
        self.gripper_opened = self.gripper_status.gPO
        self.info_msg = "Calibrating open gripper value: " + str(self.gripper_opened)
        self.loginfo_magenta(self.info_msg)
        #close gripper
        self.close()
        #read real closed gripper value
        self.gripper_closed = self.gripper_status.gPO
        self.info_msg = "Calibrating closed gripper value: " + str(self.gripper_closed)
        self.loginfo_magenta(self.info_msg)
        #open gripper
        self.open()
        #calibrate gripper range
        self.gripper_range = self.gripper_closed - self.gripper_opened
        self.tick_per_dist = self.gripper_range / self.max_gap_mm

        self.loginfo_magenta("Calibration complete")
    
    #close gripper to gap set in mm
    #before use have to use the calib() method
    #ACCURACY CAN BE IMPROVED
    def set_gap_mm(self,mm):
        self.info_msg = "Closing gripper to gap:" + str(mm) + "mm"
        rospy.loginfo(self.info_msg)
        #calculate int value to send to gripper from desired gripper gap in mm
        position = self.tick_per_dist * (self.max_gap_mm - mm)
        position = round(position)

        self.set_gap_position(position)
    
    #close gripper to gap in values from self.gripper_opened - self.gripper_closed
    def set_gap_position(self,position):
        self.info_msg = "Sending gripper to position:" + str(position)
        #position command
        self.cmd_msg.rPR = int(position)
        if self.cmd_msg.rPR > 255:
            self.cmd_msg.rPR = 255
        if self.cmd_msg.rPR < 0:
            self.cmd_msg.rPR = 0
        #send command
        self.pub_cmd_msg()
        #wait for execution
        rospy.sleep(2)

    def inc_speed(self):
        self.info_msg = "Increasing gripper speed"
        self.cmd_msg.rSP += 25
        if self.cmd_msg.rSP > 255:
            self.cmd_msg.rSP = 255
        
        self.pub_cmd_msg()
    
    def dec_speed(self):
        self.info_msg = "Decreasing gripper speed"
        self.cmd_msg.rSP -= 25
        if self.cmd_msg.rSP < 0:
            self.cmd_msg.rSP = 0
    
        self.pub_cmd_msg()
    
    def inc_force(self):
        self.info_msg = "Increasing gripper force"
        self.cmd_msg.rFR += 25
        if cself.md_msg.rFR > 255:
            self.cmd_msg.rFR = 255
    
        self.pub_cmd_msg()

    def dec_force(self):
        self.info_msg = "Decreasing gripper force"
        self.cmd_msg.rFR -= 25
        if self.cmd_msg.rFR < 0:
            self.cmd_msg.rFR = 0
    
        self.pub_cmd_msg()