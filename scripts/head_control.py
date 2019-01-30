#!/usr/bin/python
import intera_interface
import rospy
import math
from intera_interface import CHECK_VERSION
from robot_control.srv import *

class Sawyer_Head:


    def get_neutral_base_joint_angle(self):
        angle=rospy.get_param("named_poses/right/poses/shipping")[1]
        return angle

    def __init__(self):
        """
        'Wobbles' the head
        """
        self._done = False
        self._head = intera_interface.Head()
        self._limb=intera_interface.Limb()
        # self._base_rotation=intera_interface.   
        # verify robot is enabled
        # print("Getting robot state... ")
        self._robot_state = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._robot_state.state().enabled
        # print("Enabling robot... ")
        self._robot_state.enable()
        face_forward_service = rospy.Service('face_forward', FaceForward, self.face_forward)
        rotate_head_service= rospy.Service('face_angle', FaceAngle, self.face_angle)
        # print("Running. Ctrl-c to quit")
    def get_base_joint_angle(self):
        base_joint_name=self._limb.joint_names()[0]
        return self._limb.joint_angle(base_joint_name)

    def clean_shutdown(self):
        self.set_neutral()
    
    def set_angle(self,angle):
        '''
        Sets head to the desired angle
        '''
        self._head.set_pan(angle)

    def face_angle(self, req):
        try:
            angle=req.theta
            self._head.set_pan(angle)
            return FaceAngleResponse(True)
        except:
            return FaceAngleResponse(False)

    def set_neutral(self):
        """
        Sets the head back into a neutral pose
        """
        self._head.set_pan(0.0)

    def face_forward(self, object):
        '''
        Sets head facing forward regardless of body position
        '''
        try:
            base_angle= self.get_base_joint_angle()
            head_angle= self._head.pan()
            angle=-1*base_angle
            self.set_angle(angle)
            return FaceForwardResponse(True)
        except:
            return FaceForwardResponse(False)
    
    
    

if __name__ == '__main__':
    print 'ready for head commands'
    rospy.init_node("head_rotate")

    head = Sawyer_Head()
    rospy.on_shutdown(head.clean_shutdown)
    rospy.spin()
