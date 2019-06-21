import sys
import copy
import rospy
import open_close_gripper_client0 as client
import moveit_commander
import moveit_msgs.msg
import numpy as np
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
import random

class MoveitInterface(object):
    def __init__(self):
      #  rospy.init_node('simple_node', anonymous=True)
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
        p.pose.position.z = -0.7
        scene.add_box("table", p, (2,1.5,1))

        p.pose.position.x = 0.
        p.pose.position.y = -.39
        p.pose.position.z = -0.7
        scene.add_box("wall", p, (2,.1,4))

        self.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)

        self.vert_quat = [-0.707, 0.707,0,0]
        self.orthog_quat = [1,0,0,0]
        self.quat = [0, 0, -.707, -.707]

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # client.open_close_gripper_client("open")

    def move_arm_to_coord(self, x, y, gripper="move", z=-0.17):
        # move z up
        # waypoints = []
        # wpose = self.move_group.get_current_pose().pose
        #
        # wpose.position.z = 0
        # waypoints.append(copy.deepcopy(wpose))
        #
        # (plan, fraction) = self.move_group.compute_cartesian_path(
        #                                    waypoints,   # waypoints to follow
        #                                    0.01,        # eef_step
        #                                    0.0)         # jump_threshold
        # self.move_group.execute(plan)
        #
        # move to x, y position
        #self.move_group.allow_looking(True)
        #self.move_group.allow_replanning(True)
        #print(self.move_group.get_planner_id())
        #self.move_group.set_planner_id("PRMstarkConfigDefault")
        #self.move_group.set_num_planning_attempts(100)
        #print(self.move_group.get_known_contraints())
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.00)         # jump_threshold
        self.move_group.execute(plan)

        wpose.position.x = x
        wpose.position.y = y
        wpose.orientation.x = self.vert_quat[0]
        wpose.orientation.y = self.vert_quat[1]
        wpose.orientation.z = self.vert_quat[2]
        wpose.orientation.w = self.vert_quat[3]
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z+.025
        wpose.orientation.x = self.vert_quat[0]
        wpose.orientation.y = self.vert_quat[1]
        wpose.orientation.z = self.vert_quat[2]
        wpose.orientation.w = self.vert_quat[3]
        target = copy.deepcopy(wpose)

        self.move_group.set_pose_target(target)

        self.move_group.go(wait=True)

        wpose.position.z = z
        wpose.orientation.x = self.vert_quat[0]
        wpose.orientation.y = self.vert_quat[1]
        wpose.orientation.z = self.vert_quat[2]
        wpose.orientation.w = self.vert_quat[3]
        target = copy.deepcopy(wpose)

        self.move_group.set_pose_target(target)

        self.move_group.go(wait=True)

        if not gripper == "move":
            client.open_close_gripper_client(gripper)

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z+.025
        wpose.orientation.x = self.vert_quat[0]
        wpose.orientation.y = self.vert_quat[1]
        wpose.orientation.z = self.vert_quat[2]
        wpose.orientation.w = self.vert_quat[3]
        target = copy.deepcopy(wpose)
        target = copy.deepcopy(wpose)

        self.move_group.set_pose_target(target)

        self.move_group.go(wait=True)

        self.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)

        # Calling ``stop()`` ensures tnvitiationhat there is no residual movement
        self.move_group.stop()

    def goto_home(self):
        self.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

    def place_within(self,area):
        correct = False
        while not correct:
            x = random.uniform(area[0],area[1])
            y = random.uniform(area[2],area[3])
            # TODO check if x,y overlaps with another block
            #correct = check_coords(x,y)
        self.move_arm_to_coords(x,y,gripper="open")
# TEST CODE
#moveit = MoveitInterface()
#moveit.move_arm_to_coord(0.0403220002774, 0.702360541533, gripper="move", z=-.175)
