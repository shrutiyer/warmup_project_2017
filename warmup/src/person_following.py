#!/usr/bin/env python

# IMPORTS ======================================================================

import math
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# CLASSES ======================================================================

class PersonFollowing(object):

    def __init__(self):
        rospy.init_node('wall_follow')

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

    def on_laser_received(self, laser_array):
        pass

    def get_front_angle_scans(self):
        pass

    def get_ranged_scans(self):
        pass

    def get_point_to_follow(self):
        pass

# EXECUTE ======================================================================

if __name__ == '__main__':
    node = PersonFollowing()
    node.run()
