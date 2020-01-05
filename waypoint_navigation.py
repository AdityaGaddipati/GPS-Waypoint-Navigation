#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point,PointStamped,PoseStamped
from farm_limits.msg import FarmLimits
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import actionlib
import tf2_ros
import tf2_geometry_msgs
import utm
import time

wp = []; new_wp = False
def waypoints_cb(msg):
	global wp,new_wp

	try:
		transform = tf_buffer.lookup_transform('map', 'utm', rospy.Time(0))

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		return
	wp = []
	for p in msg.point:
		tf_x,tf_y,tmp1,tmp2 = utm.from_latlon(p.data[1],p.data[0])
		pose_stamped = PoseStamped()
		pose_stamped.header.frame_id = 'utm'
		pose_stamped.pose.position.x = round(tf_x,2)
		pose_stamped.pose.position.y = round(tf_y,2)
		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

		wp.append([pose_transformed.pose.position.x,pose_transformed.pose.position.y])
		# wp.append([round(tf_x,2),round(tf_y,2)])
		new_wp = True
	print "Received waypoints"
	print wp

botX=0;botY=0
def odom_cb(odom):
	global botX,botY
	botX = odom.pose.pose.position.x
	botY = odom.pose.pose.position.y

goal = MoveBaseGoal()
goal.target_pose.pose.position.x = 0; goal.target_pose.pose.position.y = 0
wp_tolerance = 3
def send_goal(goal_pt):
	global goal,wp_tolerance
	try:
		transform = tf_buffer.lookup_transform('odom', 'map', rospy.Time(0))

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		return

	pose_stamped = PoseStamped()
	pose_stamped.header.frame_id = 'map'
	pose_stamped.pose.position.x = goal_pt[0]
	pose_stamped.pose.position.y = goal_pt[1]
	pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

	odom_point = PointStamped()
	# odom_point.header.frame_id = "odom"
	# odom_point.point.x = pose_transformed.pose.position.x
	# odom_point.point.y = pose_transformed.pose.position.y
	odom_point.header.frame_id = "map"
	odom_point.point.x = goal_pt[0]
	odom_point.point.y = goal_pt[1]
	odom_point.point.z = 0
	odom_point.header.stamp = rospy.Time.now()
	gps_pub.publish(odom_point)
	# print odom_point

	oldX = goal.target_pose.pose.position.x; oldY = goal.target_pose.pose.position.y
	newX = pose_transformed.pose.position.x; newY = pose_transformed.pose.position.y
	dist_between_goals = math.sqrt(math.pow(oldX-newX,2)+math.pow(oldY-newY,2))
	# print dist_between_goals,wp_tolerance
	if dist_between_goals>wp_tolerance:
		goal.target_pose.header.frame_id = "odom"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = newX
		goal.target_pose.pose.position.y =  newY
		goal.target_pose.pose.orientation.x = 0
		goal.target_pose.pose.orientation.y = 0
		goal.target_pose.pose.orientation.z = 0
		goal.target_pose.pose.orientation.w = 1
		client.send_goal(goal)
		print "Goal sent"
		print goal

if __name__=="__main__":
	rospy.init_node('waypoint_nav')
	rospy.Subscriber('/waypoints', FarmLimits, waypoints_cb, queue_size=10)
	rospy.Subscriber('/ekf_map/odometry/filtered', Odometry, odom_cb, queue_size=10)
	gps_pub = rospy.Publisher('/GPS_odom_point', PointStamped, queue_size=10, latch=True)

	tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	client.wait_for_server()

	while not rospy.is_shutdown():

		if new_wp:
			for i in range(len(wp)):
				distance_from_wp = 1000
				# send_goal(wp[i])
				while distance_from_wp>wp_tolerance and not rospy.is_shutdown():
					send_goal(wp[i])
					time.sleep(1)
					distance_from_wp = math.sqrt(math.pow(botX-wp[i][0],2)+math.pow(botY-wp[i][1],2))
					print "Distance from wp"
					print distance_from_wp
				client.cancel_goal()
				print "Achieved Waypoint "+str(i+1)
				print "Paused"
				time.sleep(5)
				# time.sleep(2)
			for i in reversed(range(len(wp))):
				distance_from_wp = 1000
				# send_goal(wp[i])
				while distance_from_wp>wp_tolerance and not rospy.is_shutdown():
					send_goal(wp[i])
					time.sleep(1)
					distance_from_wp = math.sqrt(math.pow(botX-wp[i][0],2)+math.pow(botY-wp[i][1],2))
					print "Distance from wp"
					print distance_from_wp
				client.cancel_goal()
				print "Achieved Waypoint "+str(i+1)
				print "Paused"
				time.sleep(5)
				# time.sleep(2)

			# new_wp = False

		time.sleep(0.5)
