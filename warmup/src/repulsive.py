#!/usr/bin/env python

# IMPORTS ======================================================================

import math
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import operator

# CLASSES ======================================================================

class ObstacleAvoiding(object):

    def __init__(self):
        # Scan the front of the robot for obstacles
        self.obstacle_range = 0.7
        self.repulsive_forces = []

        self.theta = 0.03
        self.k = 0.2

        self.odom_received = False
        self.goal_odom = (3, 0)         # Goal's (x,y) in odom
        self.goal_distance = math.sqrt(self.goal_odom[0]**2 + self.goal_odom[1]**2)
        self.goal_bl = (0, 0)           # Goal's (x,y) in base_link
        self.curr_pos_odom = (0, 0)     # Current robot's position 
        self.curr_angle = 0             # Current robot's heading in radians
        self.goal_radius = 2            # Radius within which PID control exists

        rospy.init_node('obstacle_avoidance')
        self.odom_listener = rospy.Subscriber('odom', Odometry, self.on_odom_received)
        self.laser_listener = rospy.Subscriber('/stable_scan', LaserScan, self.on_laser_received)
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 5)

        self.twist = Twist()

    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_twist()
            self.twist_publisher.publish(self.twist)
            r.sleep()

    def update_twist(self):
        """
        Calculates attractive and repulsive forces
        Updates velocities accordingly
        """
        x_rep, y_rep = self.get_force_field(self.repulsive_forces)
        x_att, y_att = self.odom_to_base_link(self.goal_odom)
        x_sum, y_sum = (5 * x_rep + x_att, 5 * y_rep + y_att)

        print x_rep, y_rep, x_att, y_att 

        mag, angle = vector_to_force((x_sum, y_sum))

        self.twist.angular.z = angle * self.theta
        self.twist.linear.x = mag * self.k

    def on_laser_received(self, laser_array):
        self.repulsive_forces = self.get_obstacle_in_range(laser_array.ranges)

    def on_odom_received(self, odom):
        # Update robot's current position 
        self.curr_pos_odom = (odom.pose.pose.position.x, odom.pose.pose.position.y)

        # Update current heading 
        self.curr_angle = math.degrees(tf.transformations.euler_from_quaternion((
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w))[2])

        if not self.odom_received:
            # Set the goal distance in odom just once
            self.odom_received = True
            self.goal_odom = self.base_link_to_odom(self.goal_odom)

    def odom_to_base_link(self, odom_point):
        """
        odom_point - tuple of (x,y) of an odom point
        return - tuple of (x,y) in base_link
        """
        # TODO: take into consideration robot's odom co-ordinates: self.curr_pos_odom
        return ((odom_point[0] - self.curr_pos_odom[0]) * math.cos(math.radians(self.curr_angle)) + 
                (odom_point[1] - self.curr_pos_odom[1]) * math.sin(math.radians(self.curr_angle)), 
                -1 * (odom_point[0] - self.curr_pos_odom[0]) * math.sin(math.radians(self.curr_angle)) + 
                (odom_point[1] - self.curr_pos_odom[1]) * math.cos(math.radians(self.curr_angle)))

    def base_link_to_odom(self, base_link_point):
        """
        base_link_point - tuple of (x,y) of an odom point
        return - tuple of (x,y) in base_link
        """
        # TODO: take into consideration robot's odom co-ordinates: self.curr_pos_odom
        return (base_link_point[0] * math.cos(math.radians(self.curr_angle)) + 
                base_link_point[1] * - math.sin(math.radians(self.curr_angle)) + self.curr_pos_odom[0], 
                base_link_point[0] * math.sin(math.radians(self.curr_angle)) + 
                base_link_point[1] * math.cos(math.radians(self.curr_angle)) + + self.curr_pos_odom[1])

    def get_obstacle_in_range(self, scans):
        """
        scans - array of points from 0 to 360 with distance
        return - (force, angle) of points in that are within the range

        force = self.obstacle_range/distance
        """
        return [(self.obstacle_range/distance, angle) for angle, distance in enumerate(scans)
            if ((distance > 0) and (distance < self.obstacle_range))]

    def get_force_field(self, repulsive_forces):
        """
        repulsive_forces - (force, angle) of points in that are obstacles within the range
        return - a average force vector in the form (x_avg, y_avg) 
        """

        force_sum = (0, 0)
        avg_factor = len(repulsive_forces)+1

        for f in repulsive_forces:
            force_sum = tuple(map(operator.sub, force_sum, force_to_vector(f)))

        return (force_sum[0]/avg_factor, force_sum[1]/avg_factor)


# HELPER FUNCTIONS =============================================================

def force_to_vector(f):
    """
    convert (magnitude, angle) to (x, y)
    """
    return (f[0] * math.cos(math.radians(f[1])), 
            f[0] * math.sin(math.radians(f[1])))

def vector_to_force(v, bias=0.00001):
    """
    convert (x, y) to (magnitude, angle) 
    """
    return (math.sqrt(v[0]*v[0]+v[1]*v[1]), 
            math.degrees(math.atan2(v[1],(v[0]+bias)))) # atan2 returns an angle in range (-pi, pi)

# EXECUTE ======================================================================

if __name__ == '__main__':
    node = ObstacleAvoiding()
    node.run()
