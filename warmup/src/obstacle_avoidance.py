#!/usr/bin/env python

# IMPORTS ======================================================================

import math
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# CLASSES ======================================================================

class ObstacleAvoiding(object):

    def __init__(self):
        # Scan the front of the robot for obstacles
        self.front_angle_range = 90
        self.distance_range = (0.4, 3)  # Tuple of (min, max)
        self.odom_received = False

        self.goal_odom = (1, 0)   # Tuple of goal's (x, y) in odom
        self.goal_bl = (0, 0)     # Goal in base_link
        self.curr_pos_odom = (0, 0)
        self.curr_angle = 0       # Current angle in radians
        self.goal_radius = 2

        rospy.init_node('obstacle_avoidance')
        self.odom_listener = rospy.Subscriber('odom', Odometry, self.on_odom_received)
        self.laser_listener = rospy.Subscriber('/stable_scan', LaserScan, self.on_laser_received)
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 5)

        self.twist = Twist()

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.update_twist()
            self.twist_publisher.publish(self.twist)
            r.sleep()

    def update_twist(self):
        # Calculates attractive forces
        # Updates velocities according
        attractive_twist = self.attractive_forces()
        self.twist.linear.x = attractive_twist[0]
        self.twist.angular.z = attractive_twist[1]

    def attractive_forces(self):
        goal_distance = math.sqrt(
            (self.goal_odom[0] - self.curr_pos_odom[0])**2 +
            (self.goal_odom[1] - self.curr_pos_odom[1])**2)

        print("Goal Distance: ", goal_distance)
        self.goal_bl = self.odom_to_base_link(self.goal_odom)
        goal_angle = math.atan2(self.goal_bl[1], float(self.goal_bl[0]))
        angular = goal_angle / (2*math.pi)

        if self.goal_radius <= goal_distance:
            linear = 1
        elif goal_distance < self.goal_radius:
            linear = goal_distance / self.goal_radius

        return (linear,angular)

    def odom_to_base_link(self, odom_point):
        # Receives a tuple of odom
        # TODO: take into consideration robot's odom co-ordinates
        return (
            odom_point[0] * math.cos(self.curr_angle) + odom_point[1] * math.sin(self.curr_angle),
            -1 * odom_point[0] * math.sin(self.curr_angle) + odom_point[1] * math.cos(self.curr_angle))

    def base_link_to_odom(self):
        pass

    def on_odom_received(self, odom):
        self.curr_pos_odom = (odom.pose.pose.position.x,odom.pose.pose.position.y)
        if not self.odom_received:
            # Set the goal distance in odom
            self.goal_odom = (odom.pose.pose.position.x+self.goal_odom[0],
                odom.pose.pose.position.y+self.goal_odom[1])
            self.odom_received = True
        # Update current heading 
        self.curr_angle = tf.transformations.euler_from_quaternion((
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w))[2]

    def on_laser_received(self, laser_array):
        pass

# EXECUTE ======================================================================

if __name__ == '__main__':
    node = ObstacleAvoiding()
    node.run()
