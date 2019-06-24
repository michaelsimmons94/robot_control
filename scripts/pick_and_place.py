#!/usr/bin/env python

import sys
from robot_control.srv import *
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import moveit_interface
import open_close_gripper_client

class pick_and_place:
    
    def __init__(self):
        #self.blockToPixel=rospy.ServiceProxy('vision/blockToPixel', BlockToPixel)
        #self.pixelToCam=rospy.ServiceProxy('vision/pixelToCamera', PixelToPoint)
        #self.camToRobot=rospy.ServiceProxy('vision/cameraToPoint', PointToPoint)
        #self.interface = moveit_interface.MoveitInterface()
        #self.holdingBlock=rospy.ServiceProxy('gripper/actuate', GripperGrasping)
        pass

    def pnp(self, req):
        try:
            # Get the robots claiming area. my_area([x1,x2,y1.y2])
            #pixel=self.blockToPixel(req.color, req.shape)
            #cam = self.pixelToCam(pixel.x,pixel.y)
            #robot = self.camToRobot(cam.point)
            #if robot.point.z > -0.175:
                #robot.point.z = -0.175
            self.interface = moveit_interface.MoveitInterface()
            x = 0
            y = .7
            zed = -.175
            moveit_commander.roscpp_initialize(sys.argv)
            #print("here")
            #gripper_open = self.holdingBlock()
            #print("here")
            #while gripper_open.isOpen:
            self.interface.move_arm_to_coord(x,y,z=zed,gripper="close")
                #self.interface.move_arm_to_coord(robot.point.x,robot.point.y,z=robot.point.z,gripper="close")
                #gripper_open = self.holdingBlock()
            my_area = [-.5,-.2,0.5,0.9]
            self.interface.place_within(my_area)
            return BlockToPixelResponse(True)
        except Exception as e:
            print("ERROR:: "+str(e))
            return BlockToPixelResponse(False)

def pick_and_place_service():
    rospy.init_node('pick_and_place_service')
    pp = pick_and_place()
    s = rospy.Service('arm/pick_and_place', PickAndPlace, pp.pnp)
    print('Ready to claim blocks.')
    rospy.spin()

if __name__ == "__main__":
    pick_and_place_service()

