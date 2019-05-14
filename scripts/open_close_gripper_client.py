#!/usr/bin/env python

import sys
import rospy
from robot_control.srv import *

def actuate_gripper_client(action):
    rospy.wait_for_service('actuate_gripper')
    try:
        open_gripper = rospy.ServiceProxy('actuate_gripper', OpenGripper)
        resp = open_gripper(action)
        return resp.isOpen
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s ['open'|'close']"%sys.argv[0]

if __name__ == "__main__":
    print("THIS IS NO LONGER SUPPORTED. INSTEAD USE A ROSSERVICE CALL")
    return
