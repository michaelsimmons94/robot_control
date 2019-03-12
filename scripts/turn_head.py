#!/usr/bin/env python
import intera_interface
import os
import sys
import rospy
import curses
from robot_control.msg import TurnHead
sys.path.append(os.path.join(os.path.dirname(__file__), '../..', 'scripts'))
import helper_scripts

pub = rospy.Publisher('/head/turn', TurnHead, queue_size=10)
rospy.init_node('head_turner')
head = intera_interface.Head()
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

def turn_with_arrow_keys(direction):
        head_angle= head.pan()
        angle = head_angle + direction*.05
        return angle_error_checker(angle)

def angle_error_checker(radian):
    if radian > 0.896:
            radian = -5.14
    elif radian < -5.14:
        radian = 0.896
    return radian

try:
    while True:
        char = screen.getch()
        if char == ord('q'):
            break
        elif char == curses.KEY_RIGHT:
            angle = turn_with_arrow_keys(1)
            head.set_pan(angle)
        elif char == curses.KEY_LEFT:
            angle = turn_with_arrow_keys(-1)
            head.set_pan(angle)
        

finally:
    # shut down cleanly
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
