#!/usr/bin/env python

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
        is_clicksmart = isinstance(gripper, SimpleClickSmartGripper)

        if is_clicksmart:
            if gripper.needs_init():
                gripper.initialize()
        else:
            if not (gripper.is_calibrated() or
                    gripper.calibrate() == True):
                raise
            return
        if(req.command=="open"):
            try:
                gripper.set_ee_signal_value('grip', True)
                print "Gripper is open"
            except Exception as e:
                print e
            return OpenGripperResponse(True)
        elif(req.command=="close"):
            try:
                gripper.set_ee_signal_value('grip', False)
                print "Gripper is closed"
            except Exception as e:
                print e
            return OpenGripperResponse(False)
    except:
        gripper = None
        is_clicksmart = False

    else:
        raise rospy.ServiceException("invalid command")


def open_close_gripper_server():

    rospy.init_node('open_close_gripper_service')


    s = rospy.Service('gripper/actuate', OpenGripper, handle_request)
    print "Ready to open and close gripper."
    rospy.spin()

if __name__ == "__main__":
    open_close_gripper_server()
