import rospy
import intera_interface

rospy.init_node("Get_Joint_Angles")

limb = intera_interface.Limb('right')

print limb.joint_angles()
