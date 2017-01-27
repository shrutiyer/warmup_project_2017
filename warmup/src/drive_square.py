#!/usr/bin/env python

# IMPORTS ======================================================================

import copy
from enum import Enum
import math
import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from visualization_msgs.msg import Marker

from marker import MarkerMaker # TODO: Deprecate.
from models.position import Position

# CLASSES ======================================================================

class States(Enum):
    CALC_FORWARD = 0
    MOVE_FORWARD = 1
    CALC_TURN = 2
    TURN = 3
    STOP = 4

class SquareDrivingController(object):

    def __init__(self):
        self.current_state = States.CALC_FORWARD
        self.curr_pos = Position(0, 0)
        self.target_pos = Position(0, 0)
        self.curr_angle = 0
        self.target_angle = 0
        self.theta_pos = 0.2
        self.theta_angle = 8 # In degrees

        # TODO: Deprecate.
        self.is_callbacked = False

        rospy.init_node('drive_square')

        self.odom_listener = rospy.Subscriber('odom', Odometry, self.on_odom_received)
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

        self.twist_forward = Twist()
        self.twist_forward.linear.x = 1; self.twist_forward.linear.y = 0; self.twist_forward.linear.z = 0
        self.twist_left = Twist()
        self.twist_left.angular.z = 0.2
        self.twist_stop = Twist()

        # TODO: Deprecate
        self.marker_maker = MarkerMaker()

    def on_odom_received(self, odom):
        self.curr_pos.x = odom.pose.pose.position.x
        self.curr_pos.y = odom.pose.pose.position.y
        self.curr_angle = tf.transformations.euler_from_quaternion((
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w))[2] * 180 / (math.pi)

        # TODO: Deprecate.
        self.is_callbacked = True

    def is_in_position(self):
        error = math.sqrt((self.target_pos.x - self.curr_pos.x)**2
            + (self.target_pos.y - self.curr_pos.y)**2)
        return error < self.theta_pos

    def is_in_angle(self):
        error = math.fabs(self.target_angle - self.curr_angle)
        return error < self.theta_angle

    def set_target_pos(self):
        self.target_pos.x = self.curr_pos.x + 1 * math.cos(math.radians(self.curr_angle))
        self.target_pos.y = self.curr_pos.y + 1 * math.sin(math.radians(self.curr_angle))

    def set_target_angle(self):
        self.target_angle = ((self.curr_angle + 180 + 90) % 360) - 180

    def run(self):
        print "running"
        r = rospy.Rate(1000)
        while not rospy.is_shutdown():
            if not self.is_callbacked:
                r.sleep()
                continue

            # State machine.
            if self.current_state == States.CALC_FORWARD:
                print "calculating"
                self.set_target_pos()
                self.marker_maker.set_position(self.target_pos)
                self.marker_maker.publish()

                self.current_state = States.MOVE_FORWARD
                self.twist_publisher.publish(self.twist_forward)

            elif self.current_state == States.MOVE_FORWARD:
                print "moving forward"
                if self.is_in_position():
                    self.current_state = States.CALC_TURN
                    self.twist_publisher.publish(self.twist_stop)

            elif self.current_state == States.CALC_TURN:
                self.set_target_angle()
                self.current_state = States.TURN
                self.twist_publisher.publish(self.twist_left)

            elif self.current_state == States.TURN:
                if self.is_in_angle():
                    self.current_state = States.CALC_FORWARD
                    self.twist_publisher.publish(self.twist_stop)

            elif current_state == States.STOP:
                self.twist_publisher.publish(self.twist_stop)
                print "Stopped."

            else:
                print "Done."

            r.sleep()

# EXECUTE ======================================================================

if __name__ == '__main__':
    controller = SquareDrivingController()
    controller.run()
