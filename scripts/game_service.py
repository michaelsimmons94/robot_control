import sys
from robot_control.srv import *
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def handle_request(req):
    if req.game == "demo_game":
        my_turn = True
        game_not_done = True
        while(game_not_done):
            my_turn = True
            if my_turn:
                get_move()
                move()
                evaulate()
            else:
                observe()
            game_not_done = is_game_done()
        do_end_game_stuff()


def game_service():
    rospy.init_node('game_service')
    s = rospy.Service('game', GameInit, handle_request)
    print "Ready to play the game."
    rospy.spin()

if __name__ == "__main__":
    game_service()

