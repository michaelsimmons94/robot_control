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


class Goto_XYZ_Service(object):

    def __init__(self):
        rospy.init_node('set_up_game_node', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander('right_arm')
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # This value changes the speed of the joint
        #self.move_group.set_max_velocity_scaling_factor(1)

        # set up coord system points
        self.z = -0.16993145718


        self.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

    def reachable(self,x,y):
        if len(self.solution) > 0:
            limit = self.solution[0]
            check = (y-self.solution[1]*x)-(self.solution[2]*(x**2))
            if not check <= limit or not y >= 0:
                return False
        return True

    def move_to(self, x, y, z = -8):

        if z == -8:
            z = self.z

        try:
            # convert points to our grid system

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
            self.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()

            # move x and y
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

            #move down
            waypoints = []
            wpose = self.move_group.get_current_pose().pose

            wpose.position.z = self.z
            waypoints.append(copy.deepcopy(wpose))

            (plan, fraction) = self.move_group.compute_cartesian_path(
                                               waypoints,   # waypoints to follow
                                               0.01,        # eef_step
                                               0.0)         # jump_threshold
            self.move_group.execute(plan)
        except Exception:
            print("Failed to place, unreachable")
            raise


def main():
    print ("Creating Simple_Test")
    demo = Goto_XYZ_Service()
    demo.move_to(0,.3)
    demo.move_group.go([1.234236328125, -0.679, -2.884623046875, -1.699453125, -0.3081787109375, -0.5761552734375, -2.907107421875], wait=True)
    demo.move_to(0.362773248944, 0.89279576077, 0.0268563286235)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
       pass
