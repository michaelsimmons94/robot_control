#!/usr/bin/env python

import sys
from robot_control.srv import *
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_interface import MoveitInterface


#potentially stick handle request in a class like gripper cuff control with an init method.
def handle_request(req):

    try:
        if req.mood == "happy":
            interface = MoveitInterface()
            disco(interface)
        else:
            raise Exception("invalid mood")

        return RobotMoodResponse(True)
    except:
        return RobotMoodResponse(False)


def robot_mood_service():
    rospy.init_node('robot_mood_service')
    s = rospy.Service('robot_mood', RobotMood, handle_request)
    print "Ready to change moods."
    rospy.spin()

def disco(interface):
    interface.strict_movement(-.3,.7,z=0)
    interface.strict_movement(.3,.7,z=.5)
    interface.strict_movement(-.3,.7,z=0)
    interface.strict_movement(.3,.7,z=.5)
    interface.strict_movement(-.3,.7,z=0)
    interface.strict_movement(.3,.7,z=.5)
    interface.goto_home()

if __name__ == "__main__":
    robot_mood_service()
