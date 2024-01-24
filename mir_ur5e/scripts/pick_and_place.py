#! /usr/bin/env python

import rospy
from mir_ur5e.mir100_class import MiR100
from mir_ur5e.ur5e_class import UR5e
from move_base_msgs.msg import MoveBaseActionResult


def pick_and_place(data):
    if data.status.status == 3:
        print("END GOAL REACHED")
        print("STARTING MANIPULATOR MOVEMENT")

        manipulator = UR5e("manipulator")
        manipulator.pick_and_place()
        del manipulator

        rospy.signal_shutdown("motion concluded")
    else:
        pass

def main():

    manipulator = UR5e("manipulator")
    manipulator.pick_and_place()
    del manipulator

    rospy.signal_shutdown("motion concluded")

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)