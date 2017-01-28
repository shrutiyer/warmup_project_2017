#!/usr/bin/env python

# IMPORTS ======================================================================

import math
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# CLASSES ======================================================================

class WallFollowing(object):

    def __init__(self):
        self.min_value_angle = 0    # Angle pointing at min distance
        self.k = 2                  # Proportionality constant
        
        rospy.init_node('wall_follow')
        self.laser_listener = rospy.Subscriber('/scan', LaserScan, self.on_laser_received)
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        
        self.twist_move = Twist()   # Twist object to make neato move in world
        self.twist_stop = Twist()

    def on_laser_received(self, laser_array):
        self.curr_dist = min([distance for distance in laser_array.ranges if distance != 0])    # Min non-zero distance
        self.min_value_angle = math.radians(laser_array.ranges.index(self.curr_dist))           # Angle of that min distance

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            print "Running"
            self.update_twist()
            self.twist_publisher.publish(self.twist_move)
            r.sleep()
        print "Done"

    def update_twist(self):
        self.twist_move.angular.z = - *math.cos(self.min_value_angle % math.pi)
        self.twist_move.linear.x = abs(math.sin(self.min_value_angle))
        # print "Angle: ", self.min_value_angle, " Linear: ", self.twist_move.linear.x, " Angular: ", self.twist_move.angular.z


# EXECUTE ======================================================================

if __name__ == '__main__':
    node = WallFollowing()
    node.run()
