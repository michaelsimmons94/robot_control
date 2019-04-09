#!/usr/bin/python
import intera_interface
import rospy
import math
import os
import cv2
import cv_bridge
from intera_interface import CHECK_VERSION
from robot_control.srv import *
from robot_control.msg import *
from sensor_msgs.msg import Image

class Sawyer_Body:


    def get_neutral_base_joint_angle(self):
        angle=rospy.get_param("named_poses/right/poses/shipping")[1]
        return angle

    def __init__(self):
       
        self._done = False
        self._head = intera_interface.Head()
        self._limb=intera_interface.Limb()
        self._robot_state = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._robot_state.state().enabled
        self._robot_state.enable()
        self._display = intera_interface.HeadDisplay()
        self._last_angles = self._limb.joint_angles()
        self._last_head_pan = self._head.pan()

        move_arm_out_of_way_service = rospy.Service('arm/move_away', MoveArm, self.move_arm_out_of_way)
        return_to_last_position= rospy.Service('arm/return_to_position', MoveArm, self.return_to_last_position)
        move_to_rest = rospy.Service('arm/move_to_rest', MoveArm, self.move_to_rest)
        # arrow_key_control_servie = rospy.Service('head/arrow_key_control', PanToAngle, self.turn_with_arrow_keys)
        #lock_head_service= rospy.Service('head/lock_head', LockHead, self.lock_head)
        # turn_head=rospy.Subscriber("head/turn",TurnHead, self.turn_head)
        # display_face = rospy.Publisher('head/display', Image, latch=True, queue_size=10)

    def set_angle(self,angle):
        '''
        Sets head to the desired angle
        0 - left
        pi/2 - forward
        pi - right
        '''
        self._head.set_pan(angle)

    def get_base_joint_angle(self):
        base_joint_name=self._limb.joint_names()[0]
        return self._limb.joint_angle(base_joint_name)


    def face_forward(self):
        '''
        Sets head facing forward regardless of body position
        '''
        try:
            base_angle= self.get_base_joint_angle()
            head_angle= self._head.pan()
            angle=-1*base_angle+ math.pi/2
            self.set_angle(angle)
            return FaceForwardResponse(True)
        except:
            return FaceForwardResponse(False)


    def pan_to_angle(self, angle):
        self._head.set_pan(angle)

    def return_to_last_position(self, req):
        try:

            self._limb.move_to_joint_positions(self._last_angles)
            self.pan_to_angle(self._last_head_pan)
            return MoveArmResponse(True)
        except:
            return MoveArmResponse(False)

    def move_arm_out_of_way(self, object):
        try:
            arms_out_of_way = {'right_j0': 2.2}
            self._last_angles = self._limb.joint_angles()
            self._last_head_pan = self._head.pan()

            self._limb.move_to_joint_positions(arms_out_of_way)
            self.face_forward()
            return MoveArmResponse(True)
        except:
            return MoveArmResponse(False)

    def move_to_rest(self, req):
        self._limb = intera_interface.Limb('right')
        angle = {'right_j6': -2.907107421875, 'right_j5': -0.5761552734375, 'right_j4': -0.3081787109375, 'right_j3': -
                1.699453125, 'right_j2': -2.884623046875, 'right_j1': -0.679, 'right_j0': 1.234236328125}
        self._limb.move_to_joint_positions(angle)
        self.face_forward()




    

if __name__ == '__main__':
    print('ready for positioning commands')
    rospy.init_node("arm")

    head = Sawyer_Body()
    # rospy.on_shutdown(head.clean_shutdown)
    rospy.spin()
