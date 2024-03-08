import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class robotiq_2f85:
    """simple robotiq 2f85 gripper class
    """

    def __init__(self):
        rospy.init_node('gripper_control_python_node', anonymous=True)
        #check namespace
        if rospy.has_param('/robot_namespace_prefix'):
            self.namespace = rospy.get_param('/robot_namespace_prefix')
            if self.namespace == "":
                self.namespace = "/"
        #command publisher
        self.cmd_pub = rospy.Publisher(self.namespace + "gripper_control/output", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        #gripper status
        self.gripper_status = "Reset needed"

    def reset(self):
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_status = "Resetting gripper"
        self.cmd_msg.rACT = 0

        self.pub_cmd_msg()

    def activate(self):
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_status = "Activating gripper"
        self.cmd_msg.rACT = 1
        self.cmd_msg.rGTO = 1
        self.cmd_msg.rSP = 255
        self.cmd_msg.rFR = 150

        self.pub_cmd_msg()

    def close(self):
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_status = "Closing gripper"
        self.cmd_msg.rPR = 255

        self.pub_cmd_msg()

    def open(self):
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_status = "Opening gripper"
        self.cmd_msg.rPR = 0
        
        self.pub_cmd_msg()

    def inc_speed(self):
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_status = "Increasing gripper speed"
        self.cmd_msg.rSP += 25
        if self.cmd_msg.rSP > 255:
            self.cmd_msg.rSP = 255
        
        self.pub_cmd_msg()


    def gen_cmd_msg(self,command):
        self.cmd_msg = outputMsg.Robotiq2FGripper_robot_output()
        #------------------------
        #faster
        if command == "f":
            self.gripper_status = "Increasing gripper speed"
            self.cmd_msg.rSP += 25
            if self.cmd_msg.rSP > 255:
                self.cmd_msg.rSP = 255
        #slower
        if command == "l":
            self.gripper_status = "Decreasing gripper speed"
            self.cmd_msg.rSP -= 25
            if self.cmd_msg.rSP < 0:
                self.cmd_msg.rSP = 0
        #increase force
        if command == "i":
            self.gripper_status = "Increasing gripper force"
            self.cmd_msg.rFR += 25
            if cself.md_msg.rFR > 255:
                self.cmd_msg.rFR = 255
        #decrease force
        if command == "d":
            self.gripper_status = "Decreasing gripper force"
            self.cmd_msg.rFR -= 25
            if self.cmd_msg.rFR < 0:
                self.cmd_msg.rFR = 0
        #------------------------
        # 0-255 go to position
        if type(command) == int:
            try:
                self.gripper_status = "Sending gripper to position:" + str(command)
                self.cmd_msg.rPR = int(command)
                if self.cmd_msg.rPR > 255:
                    self.cmd_msg.rPR = 255
                if self.cmd_msg.rPR < 0:
                    self.cmd_msg.rPR = 0
            except ValueError:
                pass

        return cmd_msg

    def pub_cmd_msg(self):
        print(self.gripper_status)
        self.cmd_pub.publish(self.cmd_msg)

    def main():
        gripper = robotiq_2f85()
        gripper.