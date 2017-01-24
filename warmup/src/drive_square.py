#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import tf
import rospy

rospy.init_node('drive_square')

def on_odom_received(odom):
    yaw = tf.transformations.euler_from_quaternion((
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w))[2]
    print yaw

listener = rospy.Subscriber('/odom', Odometry, on_odom_received)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()
