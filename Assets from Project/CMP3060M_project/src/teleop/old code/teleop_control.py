#!/usr/bin/env python

# http://www.theconstructsim.com/read-laserscan-data/ -- basic scan reading
# https://github.com/risckaust/risc-documentations/blob/master/src/ros-basics/mini_project.py
# https://github.com/PacktPublishing/ROS-Robotics-By-Example-Second-Edition/blob/master/Chapter08/
# Turtlesim_joy_code/turtlesim_joy.py -- also ref the book
# https://github.com/osrf/rosbook/blob/master/teleop_bot/keys_to_twist_parameterized.py -- parameters
# http://wiki.ros.org/joy -- ROS joy page

import rospy

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import Joy, LaserScan

cmd_vel_rate = [0.6, 0.8] # set rates of speed for the turtlebot.


def teleop_callback(tele_msg):

	# this is the topic that will be published to cmd_vel_mux.
	cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
	# Send twist messages of user input to joy node.
	tele_cmd_vel = Twist()
	# left stick up and down.
	tele_cmd_vel.linear.x = tele_msg.axes[1] * cmd_vel_rate[0] # 0.6
	# left stick left and right.
	tele_cmd_vel.angular.z = tele_msg.axes[0] * cmd_vel_rate[1] # 0.8

	# values are printed to the terminal whenever linear or angular.
	rospy.loginfo('twist.linear: %f ; angular %f', tele_cmd_vel.linear.x, tele_cmd_vel.angular.z)

	# publish cmd_vel move command to Turtlesim.
	cmd_vel.publish(tele_cmd_vel)


def obstacle_proximity(scan_msg):

	# vibrate functionality in PS3 only
	if scan_msg.ranges[0] < 0.9:
		cmd_vel_rate[1] / 2  
		rospy.loginfo('WARNING: collision imminent!!!')
		#  add controller rumble -- strong

	if scan_msg.ranges[639] < 0.9:
		rospy.loginfo('WARNING: collision imminent!!!')
		#  add controller rumble -- strong

	if scan_msg.ranges[0] >= 1 and scan_msg.ranges[0] < 2:
		rospy.loginfo('CAUTION: approaching obstacle...')
		#  add controller rumble -- weak

	if scan_msg.ranges[639] >= 1 and scan_msg.ranges[639] < 2:
		rospy.loginfo('CAUTION: approaching obstacle...')
		#  add controller rumble -- weak 


def teleop_listener():

	# intiate the ros node, which is called by the launch file.
	rospy.init_node('teleop_node', anonymous=True)
	# subscribes to the newly installed joy node.
	teleop_sub = rospy.Subscriber('joy', Joy, teleop_callback, queue_size=1)
	detect_sub = rospy.Subscriber('scan', LaserScan, obstacle_proximity)

	# loops the script to continuously receive input.
	rospy.spin()


if __name__ == '__main__':

	# The code will continuously loop until CTRL+C has been pressed.
	try:
		teleop_listener()
	except rospy.ROSInterruptException:
		pass
