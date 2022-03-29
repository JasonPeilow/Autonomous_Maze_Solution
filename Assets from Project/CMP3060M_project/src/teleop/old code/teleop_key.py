#!/usr/bin/env python

"""

------------------------------- REFERENCES ---------------------------------

Quigley, M., Gerkey, B. and Smart, W.D.(2015) Programming Robots with ROS,
Sebastapol, USA: O' Reilly.

----------------------------------------------------------------------------

"""
import math
import rospy # Imported whenever a ROS node is being written
import sys # sys = system specific functions and parameters.
import select # select = access provided by a module for I/O monitoring functions.
import tty # tty = Terminal cotrol functions (TeleType). I/O control (Requires termios module).
import termios # termios = This module provides an interface for Unix Terminal Control Facilities.

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class teleop_key_control():
	def __init__(self):

		self.user_input = rospy.Publisher('button_pressed', String, queue_size = 1) # Key publisher
		self.teleop_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		rospy.Subscriber('button_pressed', String, teleop_callback, teleop_callback)

		self.tele_w = Twist()

	def user_input(self):


	# CONFIRM A REFERENCE FOR THIS.

	# Remember MUX is a mutiplexer. Originally in Groovy, but works in Kinetic.

		teleop_dpad = { 
						
			'q': [ 1, 1], 'w': [ 0, 1], 'e': [-1,  1], 
			'a': [ 1, 0], 'x': [ 0, 0], 'd': [-1,  0], 
			'z': [-1,-1], 's': [ 0,-1], 'c': [ 1, -1]	}

		# Explain how this uses radians / geometry msgs

		# INSERT DRIVERS HERE?

		twist_v = [0.7, 0.3] # array. [Linear, Angular]

	def teleop_callback(self, msg, teleop_pub):

		if len(msg.data) == 0 or not teleop_dpad.has_key(msg.data[0]):
			return

			dpad_input = teleop_dpad[msg.data[0]]

			tele_w.angular.z = dpad_input[0] * twist_v[0] 
			tele_w.linear.x  = dpad_input[1] * twist_v[1]

			teleop_pub.publish(tele_w)

		# (Quigley et al., 2015, PAGE)

if __name__ == '__main__': 

	rospy.init_node('tele_control', anonymous = True)

	freq = rospy.Rate(100) 

	key_pad = teleop_key_control()
		
		# Termios library is used to capture user input and calls from tty for I/O control
		# tcgetattr (GET) -- A list containing tty values is returned (captured/saved user input)
		# sys.stdin -- will iterate through the input until the 'CTRL+C' is pressed
		# setcbreak -- in Linux, cbreak disables line buffering and ...
		# fileno -- ... returns int value from the arguement buffer.
		stored_input = termios.tcgetattr(sys.stdin)
		tty.setcbreak(sys.stdin.fileno()) 
		
		print "User input is being published. CTRL+C to stop teleop control"
	 
		while not rospy.is_shutdown(): # if statement is executed until flag is true (CTRL+C)
			if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
						user_input.publish(sys.stdin.read(1))
				freq.sleep()

		# tcsetattr (SET) -- tty value is set by tcgetattr (return_attr) and ...
		# TSCADRAIN -- ... attributes are to be changed after transmitting the queued output (User input)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, stored_input)

		rospy.spin()
		
	# (Quigley et al., 2015, PAGE)

# REF Ch 8: Programming Robots with ROS
