#!/usr/bin/env python

"""
__________________________________________________________________________________________________________
References
__________________________________________________________________________________________________________

Quigley, M., Gerkey, G. and Smart, W.D. (2015) Programming Robots with ROS.
Sebastopol, USA: O'Reilly Media, Inc.

Open Source Robotics Foundation (2015) Github. osrf/rosbook. Available from
https://github.com/osrf/rosbook/blob/master/teleop_bot/key_publisher.py [accessed 9 February 2020].

__________________________________________________________________________________________________________
Brief Description
__________________________________________________________________________________________________________





"""
# This has only been used to attempt some basic teleop commands on the PWC (wheelchair)

# BEGIN ALL
import sys, select, tty, termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	key_pub = rospy.Publisher('keys', String, queue_size=1)
	rospy.init_node("keyboard_driver")
	rate = rospy.Rate(100)
	# BEGIN TERMIOS
	old_attr = termios.tcgetattr(sys.stdin)
	tty.setcbreak(sys.stdin.fileno())
	# END TERMIOS
	print "Publishing keystrokes. Press Ctrl-C to exit..."
	while not rospy.is_shutdown():
		# BEGIN SELECT
		if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
			key_pub.publish(sys.stdin.read(1))
		rate.sleep()
		# END SELECT
	# BEGIN TERMIOS_END
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
	# END TERMIOS_END
# END ALL

# (Quigley et al., 2015, 112-114)
# (OSRF, 2015)