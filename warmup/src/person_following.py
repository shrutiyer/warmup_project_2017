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
        self.distance_range = (0.3, 1.3)
        self.point_to_follow = (0, 0) # Tuple of distance, angle.
        rospy.init_node('person_follow')
        self.laser_listener = rospy.Subscriber('/stable_scan', LaserScan, self.on_laser_received)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

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
        return an array with corrected angles
        """
        for i in range(len(range_angle_filtered_scan)):
            if range_angle_filtered_scan[i][1] > 180:
                range_angle_filtered_scan[i] = (range_angle_filtered_scan[i][0], range_angle_filtered_scan[i][1]-360)

        return range_angle_filtered_scan

    def get_point_to_follow(self, angle_corrected_scan):
        """
        angle_corrected_scan - (distance, angle) of points in that are within the angle and distance range
        return center of mass of (distance, angle) 
        """
        return (sum([distance for distance, angle in angle_corrected_scan])/float(len(angle_corrected_scan)+1), 
            sum([angle for distance, angle in angle_corrected_scan])/float(len(angle_corrected_scan)+1))

# EXECUTE ======================================================================

if __name__ == '__main__':
    node = PersonFollowing()
    node.run()
