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
        self.point_to_follow = (0, 0) # Tuple of distance, angle.
        rospy.init_node('person_follow')
        self.laser_listener = rospy.Subscriber('/stable_scan', LaserScan, self.on_laser_received)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

    def on_laser_received(self, laser_array):
        # self.point_to_follow = self.get_point_to_follow(
        #     self.get_ranged_scans(
        #         self.get_front_angle_scans(self.front_angle_range, laser_array.ranges))))

    def get_front_angle_scans(self, front_angle_range, scans):
        """
        front_angle_range - width from front in degrees
        scans - array of points from 0 to 360 with distance
        return (distance, angle) of points in that are within the range (in degrees)
        """
        return [(distance, angle) for distance, angle in enumerate(scans)
            if (angle <= front_angle_range/2) or (angle > 359 - front_angle_range/2)]

    def get_ranged_scans(self, scans):
        pass

    def get_point_to_follow(self):
        pass

# EXECUTE ======================================================================

if __name__ == '__main__':
    node = PersonFollowing()
    node.run()
