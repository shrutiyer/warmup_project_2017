#!/usr/bin/env python

# IMPORTS ======================================================================

import math
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# CLASSES ======================================================================

class PersonFollowing(object):

    def __init__(self):
        self.front_angle_range = 90
        self.distance_range = (0.4, 2)  # Tuple of (min, max)
        self.point_to_follow = (0, 0)   # Tuple of (distance, angle)
        self.person_distance = 0.5      # distance want to keep away from the person
        # Proportionality constant
        self.theta = 2
        self.k = 2

        rospy.init_node('person_follow')
        self.laser_listener = rospy.Subscriber('/stable_scan', LaserScan, self.on_laser_received)

        self.twist = Twist()

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.update_twist()
            self.twist_publisher.publish(self.twist)
            r.sleep()

    def update_twist(self):
        self.twist.angular.z = - self.point_to_follow[1] * self.theta
        self.twist.linear.x = (self.point_to_follow[0] - self.person_distance) * self.k

    def update_marker(self):
        pass

    def on_laser_received(self, laser_array):
        self.point_to_follow = self.get_point_to_follow(
            self.angle_correct(
                self.get_ranged_scans(
                    self.distance_range, self.get_front_angle_scans(self.front_angle_range, laser_array.ranges))))

        print self.point_to_follow

    def get_front_angle_scans(self, front_angle_range, scans):
        """
        front_angle_range - width from front in degrees
        scans - array of points from 0 to 360 with distance
        return (distance, angle) of points in that are within the range (in degrees)
        """
        return [(distance, angle) for angle, distance in enumerate(scans)
            if (angle <= front_angle_range/2) or (angle > 361 - front_angle_range/2)]

    def get_ranged_scans(self, distance_range, angle_filtered_scan):
        """
        distance_range - a tuple of (min range, max range) in meters
        angle_filtered_scan - (distance, angle) of points in that are within the angle range
        return (distance, angle) of points in that are within the distance range (in meters)
        """
        return [(distance, angle) for distance, angle in angle_filtered_scan
            if (distance >= distance_range[0]) and (distance <= distance_range[1])]

    def angle_correct(self, range_angle_filtered_scan):
        """
        Convert points with angle larger than 180 to the negative range angles

        range_angle_filtered_scan - (distance, angle) of points in that are within the angle and distance range
        return an array of (distance, angle) points with corrected angles
        """
        for i in range(len(range_angle_filtered_scan)):
            if range_angle_filtered_scan[i][1] > 180:
                range_angle_filtered_scan[i] = (range_angle_filtered_scan[i][0], range_angle_filtered_scan[i][1]-360)

        return range_angle_filtered_scan

    def get_point_to_follow(self, angle_corrected_scan, bias=0.1):
        """
        angle_corrected_scan - (distance, angle) of points with corrected angles
        return center of mass of (distance, angle) 
        """

        # divide by total number of points with a bias term in case of 0
        div = len(angle_corrected_scan) + bias

        return (sum([distance for distance, angle in angle_corrected_scan]) / div, 
            sum([angle for distance, angle in angle_corrected_scan]) / div)

# EXECUTE ======================================================================

if __name__ == '__main__':
    node = PersonFollowing()
    node.run()
