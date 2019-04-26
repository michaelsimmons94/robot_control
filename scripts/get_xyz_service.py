#!/usr/bin/env python

import sys
from robot_control.srv import *
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


#potentially stick handle request in a class like gripper cuff control with an init method.
def handle_request(req):

    try:
        moveit_commander.roscpp_initialize(sys.argv)
        move_group = moveit_commander.MoveGroupCommander('right_arm')

        tokens = str(move_group.get_current_pose().pose).split('\n')
        pos_x = float(tokens[1][5:])
        pos_y = float(tokens[2][5:])
        pos_z = float(tokens[3][5:])

        print("x: ", pos_x, " y: ", pos_y, " z: ", pos_z)

        return XYZPoseResponse(pos_x, pos_y, pos_z)
    except:
        print("Failed to get xyz position")


def get_xyz_service():
    rospy.init_node('get_xyz_service')
    s = rospy.Service('get_xyz', XYZPose, handle_request)
    print "Ready to give xyz position."
    rospy.spin()

if __name__ == "__main__":
    get_xyz_service()
