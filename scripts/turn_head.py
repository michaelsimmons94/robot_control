#!/usr/bin/env python

import sys
import rospy
import curses
from robot_control.msg import TurnHead
from robot_control.srv import LockHead


pub = rospy.Publisher('/head/turn', TurnHead, queue_size=10)
rospy.init_node('head_turner')
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

try:
    while True:
        char = screen.getch()
        if char == ord('q'):
            break
        elif char == curses.KEY_RIGHT:
            screen.addstr(0, 0, 'right')
            pub.publish(False)
        elif char == curses.KEY_LEFT:
            screen.addstr(0, 0, 'left ')
            pub.publish(True)
        

finally:
    # shut down cleanly
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()



rospy.spin()