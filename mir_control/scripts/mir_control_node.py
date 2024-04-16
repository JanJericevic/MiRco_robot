#! /usr/bin/env python

import rospy
from mir_control.mir100_class import MiR100
from mir_rest_api.api import MirRestApi

from pprint import pprint

def main():
    # init node
    rospy.init_node('mir_control_node', anonymous=True)

    # init mobile base
    mir_ip = rospy.get_param("/robot_base_ip")
    mir = MiR100(use_api=True, api_uname="Distributor", api_pass="distributor", mir_ip=mir_ip)

    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException as e:
        print("error: %s" %e)
    except rospy.ROSInterruptException:
        pass