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
        self.obstacle_range = 0.6
        self.repulsive_forces = []

        self.theta = 0.005
        self.k = 0.1

        rospy.init_node('obstacle_avoidance')
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
        """
        Calculates attractive and repulsive forces
        Updates velocities accordingly
        """
        mag, angle = vector_to_force(self.get_force_field(self.repulsive_forces))
        print mag, angle

        self.twist.angular.z = angle * self.theta
        self.twist.linear.x = mag * self.k

    def repulsive_forces(self):
        """
        return - tuple of (linear, angular) from sum of repulsive forces
                 from all nearby obstacles
        """
        pass

    def on_laser_received(self, laser_array):
        self.repulsive_forces = self.get_obstacle_in_range(laser_array.ranges)

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
