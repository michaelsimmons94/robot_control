#!/usr/bin/env python

import sys
from robot_control.srv import *
import rospy
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    RobotParams,
)

#potentially stick handle request in a class like gripper cuff control with an init method.
def handle_request(req):

    try:
        gripper = get_current_gripper_interface()

        isOpen = gripper.__dict__['gripper_io'].__dict__['signals'].get('open_B1ZwR2PmT7').get('data')
        isClosed = gripper.__dict__['gripper_io'].__dict__['signals'].get('closed_H1GwChPmT7').get('data')

        print("isOpen:", isOpen)
        print("isClosed:", isClosed)

        is_clicksmart = isinstance(gripper, SimpleClickSmartGripper)

        return GripperGraspingResponse(isOpen, isClosed)
    except:
        gripper = None
        is_clicksmart = False

    else:
        raise rospy.ServiceException("invalid command")


def get_gripper_status_server():

    rospy.init_node('get_gripper_status_server')

    rospy.wait_for_service('open_close_gripper')

    try:
        oc_gripper = rospy.ServiceProxy('open_close_gripper', OpenGripper)
        oc_gripper("close")
        oc_gripper("open")
    except rospy.ServiceException:
        print "failed!"

    s = rospy.Service('get_gripper_status', GripperGrasping, handle_request)
    print "Ready to give gripper status."
    rospy.spin()

if __name__ == "__main__":
    get_gripper_status_server()
