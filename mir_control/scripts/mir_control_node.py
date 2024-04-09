#! /usr/bin/env python

import rospy
from mir_control.mir100_class import MiR100
from mir_rest_api.api import MirRestApi

def main():
    # init node
    rospy.init_node('mir_control_node', anonymous=True)

    # init mobile base
    mir_ip = rospy.get_param("/robot_base_ip")
    api = MirRestApi("Distributor","distributor", mir_ip)
    mir = MiR100(api)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)
    except rospy.ROSInterruptException:
        pass