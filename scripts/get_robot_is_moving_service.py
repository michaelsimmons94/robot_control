#!/usr/bin/env python

import sys
from robot_control.srv import *
import rospy
import intera_interface
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from time import sleep


#potentially stick handle request in a class like gripper cuff control with an init method.
def handle_request(req):
    limb = intera_interface.Limb('right')

    try:
        angles_before = limb.joint_angles()
        sleep(0.5)
        angles_after = limb.joint_angles()

        print("before: ", angles_before)
        print("after: ", angles_after)

        for key in angles_before.keys():
            if round(angles_before[key],4) != round(angles_after[key],4):
                return MoveArmResponse(True)

        print("C")

        return MoveArmResponse(False)
    except:
        print("D")
        return None

def get_robot_is_moving_service():
    rospy.init_node('robot_is_moving_service')
    s = rospy.Service('robot_is_moving', MoveArm, handle_request)
    print "Ready to determine if robot is moving."
    rospy.spin()

if __name__ == "__main__":
    get_robot_is_moving_service()
