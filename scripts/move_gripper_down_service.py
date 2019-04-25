#!/usr/bin/env python

import sys
from robot_control.srv import *
import rospy
import intera_interface

#potentially stick handle request in a class like gripper cuff control with an init method.
def handle_request(req):

    try:
        limb = intera_interface.Limb('right')
        angles = limb.joint_angles()
        # angles['right_j5'] = -0.5761552734375
        # angles['right_j6'] = -2.907107421875
        # limb.move_to_joint_positions(angles)
        print(angles)
        print("MY FUNC HERE")
        return MoveArmResponse(True)
    except:
        print("failed")
        return MoveArmResponse(False)


    else:
        raise rospy.ServiceException("invalid command")
        return MoveArmResponse(False)



def move_gripper_down_service():

    rospy.init_node('move_gripper_down_service')
    s = rospy.Service('move_gripper_down', MoveArm, handle_request)
    rospy.spin()

if __name__ == "__main__":
    move_gripper_down_service()
