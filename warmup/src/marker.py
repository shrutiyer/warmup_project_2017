#!/usr/bin/env python

""" This script is a ROS node that publish a marker object """

from visualization_msgs.msg import Marker 
from std_msgs.msg import Header

import rospy

# init the marker node
rospy.init_node('marker')

# init a publisher of topic my_marker
publisher = rospy.Publisher('/my_marker', Marker, queue_size=10)

# setup marker
my_marker = Marker()
my_marker.type = Marker.SPHERE

# Header setup
my_marker.header.frame_id = "odom"
my_marker.header.stamp = rospy.Time.now()

# set x y z positions
my_marker.pose.position.x = 1
my_marker.pose.position.y = 1
my_marker.pose.position.z = 0

# set x y z scales
my_marker.scale.x = 1
my_marker.scale.y = 1
my_marker.scale.z = 1

# rgb and alpha values
my_marker.color.r = 1.0
my_marker.color.g = 1.0
my_marker.color.b = 0.0
my_marker.color.a = 1.0

r = rospy.Rate(10) # set a publish rate of 10 HZ

while not rospy.is_shutdown():
    publisher.publish(my_marker)
    r.sleep()

print "Marker is finished"