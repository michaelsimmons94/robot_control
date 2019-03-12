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

class Sawyer_Head:


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

        face_forward_service = rospy.Service('head/face_forward', FaceForward, self.face_forward)
        rotate_head_service= rospy.Service('head/pan_to_angle', PanToAngle, self.pan_to_angle)
        # arrow_key_control_servie = rospy.Service('head/arrow_key_control', PanToAngle, self.turn_with_arrow_keys)
        #lock_head_service= rospy.Service('head/lock_head', LockHead, self.lock_head)
        turn_head=rospy.Subscriber("head/turn",TurnHead, self.turn_head)
        display_face = rospy.Publisher('head/display', Image, latch=True, queue_size=10)
        
    def get_base_joint_angle(self):
        base_joint_name=self._limb.joint_names()[0]
        return self._limb.joint_angle(base_joint_name)

    
    def set_angle(self,angle):
        '''
        Sets head to the desired angle
        0 - left
        pi/2 - forward
        pi - right
        '''
        self._head.set_pan(angle)

    def pan_to_angle(self, req):
        try:
            angle=req.theta
            self._head.set_pan(angle)
            return PanToAngleResponse(True)
        except:
            return PanToAngleResponse(False)

    def face_forward(self, object):
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
    
    def turn_head(self, req):
        try:
            head_angle=self._head.pan()
            if req.left:
                self.set_angle(head_angle+math.radians(3))
            else:
                self.set_angle(head_angle-math.radians(3))
        except:
            rospy.logwarn("can't turn head any further")

    def display(self, face):
        '''
        display a face on sawyer
        '''
        pass


    

if __name__ == '__main__':
    print('ready for head commands')
    rospy.init_node("head")

    head = Sawyer_Head()
    # rospy.on_shutdown(head.clean_shutdown)
    rospy.spin()
