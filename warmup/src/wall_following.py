#!/usr/bin/env python

# IMPORTS ======================================================================

import math
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# CLASSES ======================================================================

class WallFollowing(object):

    def __init__(self):
        self.min_value_angle = 0    # Angle pointing at min distance
        self.curr_dist = 0
        self.k = 2                  # Proportionality constant

        rospy.init_node('wall_follow')
        self.laser_listener = rospy.Subscriber('/scan', LaserScan, self.on_laser_received)
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.marker_publisher = rospy.Publisher('/wall_marker', Marker, queue_size=10)

        self.twist_move = Twist()   # Twist object to make neato move in world
        self.twist_stop = Twist()

        # setup marker
        self.curr_marker = Marker()
        self.curr_marker.type = Marker.SPHERE

        # Header setup
        self.curr_marker.header.frame_id = "base_link"
        self.curr_marker.header.stamp = rospy.Time.now()

        # set x y z positions
        self.curr_marker.pose.position.x = 0
        self.curr_marker.pose.position.y = 0
        self.curr_marker.pose.position.z = 0

        # set x y z scales
        self.curr_marker.scale.x = 0.4
        self.curr_marker.scale.y = 0.4
        self.curr_marker.scale.z = 0.4

        # rgb and alpha values
        self.curr_marker.color.r = 1.0
        self.curr_marker.color.g = 0.0
        self.curr_marker.color.b = 0.0
        self.curr_marker.color.a = 1.0

    def on_laser_received(self, laser_array):
        scans = [distance for distance in laser_array.ranges if distance != 0]
        if len(scans) > 0:
            self.curr_dist = min(scans)    # Min non-zero distance
            self.min_value_angle = math.radians(laser_array.ranges.index(self.curr_dist)) # Angle of that min distance

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.update_twist()
            self.twist_publisher.publish(self.twist_move)
            self.update_marker()
            self.marker_publisher.publish(self.curr_marker)
            r.sleep()

    def update_twist(self):
        self.twist_move.angular.z = - self.k * math.cos(self.min_value_angle % math.pi)
        self.twist_move.linear.x = abs(math.sin(self.min_value_angle))

    def update_marker(self):
        self.curr_marker.pose.position.x = self.curr_dist*math.cos(self.min_value_angle)
        self.curr_marker.pose.position.y = self.curr_dist*math.sin(self.min_value_angle)

# EXECUTE ======================================================================

if __name__ == '__main__':
    node = WallFollowing()
    node.run()
