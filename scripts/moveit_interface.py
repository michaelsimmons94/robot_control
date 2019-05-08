import sys
import copy
import rospy
import open_close_gripper_client as client
import moveit_commander
import moveit_msgs.msg
import numpy as np
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time


class MoveitInterface(object):
    def __init__(self):
        #rospy.init_node('simple_node', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander('right_arm')

        planning_frame = self.move_group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        eef_link = self.move_group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        print "============ Printing robot state"
        print robot.get_current_state()
        print ""


        print "============ Current values:"
        current_state = self.move_group.get_current_joint_values()
        print current_state

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

        rospy.sleep(2)

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        p.pose.position.x = 0.
        p.pose.position.y = 1.
        p.pose.position.z = -0.672
        scene.add_box("table", p, (2,1.5,1))

        p.pose.position.x = 0.
        p.pose.position.y = -.39
        p.pose.position.z = -0.672
        scene.add_box("wall", p, (2,.1,4))

        self.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        client.open_close_gripper_client("open")

    def move_arm_to_coord(self, x, y, gripper="move", z=-0.170):
        # move z up
        waypoints = []
        wpose = self.move_group.get_current_pose().pose

        wpose.position.z = 0
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
        self.move_group.execute(plan)

        # move to x, y position
        waypoints = []
        wpose = self.move_group.get_current_pose().pose

        wpose.position.x = x
        wpose.position.y = y
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
        self.move_group.execute(plan)

        # move z down
        waypoints = []
        wpose = self.move_group.get_current_pose().pose

        wpose.position.z = z
        target = copy.deepcopy(wpose)

        self.move_group.set_pose_target(target)

        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        if not gripper == "move":
            client.open_close_gripper_client(gripper)

        self.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

    def strict_movement(self,x,y,z):
        # move to x, y position
        waypoints = []
        wpose = self.move_group.get_current_pose().pose

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
        self.move_group.execute(plan)

    def goto_home(self):
        self.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

# TEST CODE
#moveit = MoveitInterface()
