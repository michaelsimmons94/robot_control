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
        # robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # This value changes the speed of the joint
        #self.move_group.set_max_velocity_scaling_factor(1)

        # move x and y
        waypoints = []
        wpose = move_group.get_current_pose().pose

        wpose.position.x = req.x
        wpose.position.y = req.y
        wpose.position.z = req.z

        waypoints.append(wpose)

        (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        move_group.execute(plan)

        return XYZCommandResponse(True)
    except:
        return XYZCommandResponse(False)


def goto_xyz_service():
    rospy.init_node('goto_xyz_service')
    s = rospy.Service('goto_xyz', XYZCommand, handle_request)
    print "Ready to move to xyz positions."
    rospy.spin()

if __name__ == "__main__":
    goto_xyz_service()
