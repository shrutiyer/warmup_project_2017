#!/usr/bin/env python

""" This script is a ROS node that publish a marker object """

from visualization_msgs.msg import Marker
from std_msgs.msg import Header

import rospy

class MarkerMaker:

    def __init__(self):
        # init a publisher of topic curr_marker
        self.publisher = rospy.Publisher('/marker_maker', Marker, queue_size=10)

        # setup marker
        self.curr_marker = Marker()
        self.curr_marker.type = Marker.SPHERE

        # Header setup
        self.curr_marker.header.frame_id = "odom"
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

    def set_position(self, position):
        self.curr_marker.pose.position.x = position.x
        self.curr_marker.pose.position.y = position.y

    def publish(self):
        self.publisher.publish(self.curr_marker)

if __name__ == '__main__':
    try:
        # init the marker node
        rospy.init_node('marker')
        marker_maker = MarkerMaker()
        r = rospy.Rate(10) # set a publish rate of 10 HZ
        while not rospy.is_shutdown():
            marker_maker.publish()
            r.sleep()
    except rospy.ROSInterrutException:
        pass
