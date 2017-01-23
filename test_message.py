#!/usr/bin/env python

""" This script is our first ROS node.  We'll publish some messages """

# from geometry_msgs.msg import PointStamped, Point
# from std_msgs.msg import Header
from visualization_msgs.msg import Marker 
from std_msgs.msg import Header
import rospy

rospy.init_node('test_marker')
loop_count = 0

publisher = rospy.Publisher('/my_marker', Marker, queue_size=10)

# Marker setup
my_marker = Marker()
my_marker.type = Marker.SPHERE
# Header setup
my_marker.header.frame_id = "odom"
my_marker.header.stamp = rospy.Time.now()
# x y z position and scale
my_marker.pose.position.x = 1
my_marker.pose.position.y = 1
my_marker.pose.position.z = 0
my_marker.scale.x = 1
my_marker.scale.y = 1
my_marker.scale.z = 1
# rgb and alpha values
my_marker.color.r = 1.0
my_marker.color.g = 1.0
my_marker.color.b = 0.0
my_marker.color.a = 1.0

r = rospy.Rate(10)
while not rospy.is_shutdown():
    print "looping", loop_count
    loop_count += 1
    publisher.publish(my_marker)
    r.sleep()

print "Node is finished"