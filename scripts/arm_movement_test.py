import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time


class Simple_test(object):

    def __init__(self):
        rospy.init_node('simple_node', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander('right_arm')

        planning_frame = move_group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        eef_link = move_group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        print "============ Printing robot state"
        print robot.get_current_state()
        print ""


        print "============ Current values:"
        current_state = move_group.get_current_joint_values()
        print current_state

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

        # Planning movement:
        # +X => right
        # -X => left
        # +Y => forward (out)
        # -Y => backward (in)
        # +Z => up
        # -Z => down
        waypoints = []
        scale = 5.0
        wpose = move_group.get_current_pose().pose
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
        move_group.execute(plan)


def main():
    print ("Creating Simple_Test")
    example = Simple_test()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
       pass
