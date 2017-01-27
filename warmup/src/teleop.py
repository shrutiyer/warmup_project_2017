#!/usr/bin/env python

""" This script is a ROS node that tele operate a robot """

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

actions = {
    'w':(1,0,0,0),
    'a':(0,0,0,1),
    's':(-1,0,0,0),
    'd':(0,0,0,-1),
    'k':(0,0,0,0)
}

prints = {
    'w':'moving forward',
    'a':'turning left',
    's':'moving backward',
    'd':'turning right',
    'k':'stopped'
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None

# init the node and a publisher
publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
rospy.init_node('teleop')

while key != '\x03': # looping utill 'ctrl-c'
    key = getKey()

    x = 0
    y = 0
    z = 0
    w = 0

    if key in actions:
        print prints[key]

        x = actions[key][0]
        y = actions[key][1]
        z = actions[key][2]
        w = actions[key][3]

        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y 
        twist.linear.z = z
        twist.angular.z = w

        publisher.publish(twist)

    else:
        print "Not a valid action"

