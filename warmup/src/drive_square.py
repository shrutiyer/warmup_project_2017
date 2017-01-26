#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import tf
import rospy
import math
from enum import Enum
import copy

class States(Enum):
    CALC_FORWARD = 0
    MOVE_FORWARD = 1
    CALC_TURN = 2
    TURN = 3
    STOP = 4

class Position:
    def __init__(self):
        self.x = 0
        self.y = 0

# Globals.
# TODO: Refactor.
current_state = States.CALC_FORWARD
curr_pos = Position()
curr_angle = 0
target_pos = Position()
target_angle = 0
pos_theta = 0.1 # TODO: Figure out units.
angle_theta = 5 # Degrees

def on_odom_received(odom):
    curr_pos = odom.pose.pose.position
    curr_angle = tf.transformations.euler_from_quaternion((
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w))[2] * 180 / (math.pi)

def is_in_position(curr_pos, target_pos, theta):
    error = math.sqrt((target_pos.x - curr_pos.x)**2 - (target_pos.y - curr_pos.y)**2)
    return error < theta

def is_in_angle(curr_angle, target_angle, theta):
    error = math.fabs(target_angle - curr_angle)
    return error < theta

def get_target_pos(position, heading):
    # TODO: Fix.
    new_pos = copy.deepcopy(position)
    new_pos.x += new_pos.x + 10
    return new_pos

def get_target_angle(angle):
    return ((angle + 180 + 90) % 360) - 180

publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
rospy.init_node('drive_square')
listener = rospy.Subscriber('odom', Odometry, on_odom_received)

forward_twist = Twist()
forward_twist.linear.x = 1; forward_twist.linear.y = 0; forward_twist.linear.z = 0
turn_twist = Twist()
turn_twist.angular.z = 1
stop_twist = Twist()

r = rospy.Rate(10)
while not rospy.is_shutdown():

    # State machine.
    if current_state == States.CALC_FORWARD:
        print "calcuating forward"
        target_pos = get_target_pos(curr_pos, curr_angle)
        current_state = States.MOVE_FORWARD
        publisher.publish(forward_twist)

    elif current_state == States.MOVE_FORWARD:
        print "in moving state"
        print "current:"
        print curr_pos.x
        print "target: "
        print target_pos.x
        # Check if we are in the destination.
        if is_in_position(curr_pos, target_pos, pos_theta):
            current_state = States.CALC_TURN
            publisher.publish(stop_twist)

    elif current_state == States.CALC_TURN:
        target_angle = get_target_angle(curr_angle)
        current_state = States.TURN
        publisher.publish(turn_twist)

    elif current_state == States.TURN:
        if is_in_angle(curr_angle, target_angle, angle_theta):
            current_state = States.CALC_FORWARD
            publisher.publish(stop_twist)

    elif current_state == States.STOP:
        publisher.publish(stop_twist)
        print "Stopped."

    else:
        print "Done."

    r.sleep()
