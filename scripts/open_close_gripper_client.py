#!/usr/bin/env python

import sys
import rospy
from robot_control.srv import *

def open_close_gripper_client(action):
    rospy.wait_for_service('open_close_gripper')
    try:
        open_gripper = rospy.ServiceProxy('open_close_gripper', OpenGripper)
        resp = open_gripper(action)
        return resp.isOpen
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s ['open'|'close']"%sys.argv[0]

if __name__ == "__main__":
    print("THIS IS NO LONGER SUPPORTED. INSTEAD USE A ROSSERVICE CALL")
    #return
    # if len(sys.argv) == 2:
    #     action = str(sys.argv[1])
    # else:
    #     print usage()
    #     sys.exit(1)
    # print "Requesting to %s gripper"%(action)
    # if(open_close_gripper_client(action)):
    #     print "gripper is open"
    # else:
    #     print "gripper is closed"
