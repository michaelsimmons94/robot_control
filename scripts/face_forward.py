#!/usr/bin/python
from intera_interface import Head, HeadDisplay, Limb, RobotEnable, CHECK_VERSION
import rospy
import math
from intera_core_msgs.msg import HeadState

class Sawyer_Head:


    def __init__(self):

        self._done = False
        self._head = Head()
        self._robot_state = RobotEnable(CHECK_VERSION)
        self._init_state = self._robot_state.state().enabled
        rospy.Subscriber("robot/head/head_state", HeadState, self.straighten)

    def straighten(self,headState):
        try:
            self._head.set_pan(math.pi/2,speed=1.0,timeout=1.0,active_cancellation=True)
        except OSError, e:
            pass
            # print(e)

if __name__ == '__main__':
    print('head facing forward...')
    rospy.init_node("face_forward",anonymous=True)
    head = Sawyer_Head()
    rospy.spin()