#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point,PointStamped,PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import actionlib
import tf2_ros
import tf2_geometry_msgs
import utm

rospy.init_node('GPS_goals')

tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
tf_listener = tf2_ros.TransformListener(tf_buffer)

client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
client.wait_for_server()

gps_pub = rospy.Publisher('/GPS_odom_point', PointStamped, queue_size=10, latch=True)

utm_point = PointStamped()
utm_point.header.frame_id = "utm"
odom_point = PointStamped()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "odom"
goal.target_pose.pose.orientation.x = 0
goal.target_pose.pose.orientation.y = 0
goal.target_pose.pose.orientation.z = 0
goal.target_pose.pose.orientation.w = 1

def UTM_point(msg):
	global utm_point, odom_point, goal
	print msg
	utm_point.point = msg
	utm_point.header.stamp = rospy.Time.now()

	tf_x,tf_y,tmp1,tmp2 = utm.from_latlon(msg.x,msg.y)
	#print tf_x,tf_y

	try:
		transform = tf_buffer.lookup_transform('odom', 'utm', rospy.Time(0))

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		return

	pose_stamped = PoseStamped()
	pose_stamped.header.frame_id = 'utm'
	pose_stamped.pose.position.x = tf_x
	pose_stamped.pose.position.y = tf_y
	pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

	odom_point.header.frame_id = "odom"
	odom_point.point.x = pose_transformed.pose.position.x
	odom_point.point.y = pose_transformed.pose.position.y
	odom_point.point.z = 0
	odom_point.header.stamp = rospy.Time.now()
	gps_pub.publish(odom_point)
	print odom_point

	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position =  odom_point.point
	client.send_goal(goal)
	print "Goal sent"

gps_sub = rospy.Subscriber('/GPS_points', Point, UTM_point, queue_size=10)
print "Here"
rospy.spin()
