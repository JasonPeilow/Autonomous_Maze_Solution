#!/usr/bin/env python

"""
__________________________________________________________________________________________________________
References
__________________________________________________________________________________________________________

Fairchild, C. and Harman, T.L. (2017) Ros Robotics By Example, 2nd Edition.
Birmingham, UK: Packt Publishing Ltd.

Packt Publishing (2017) Github. ROS-Robotics-By Example/Chapter08/Turtlesim_joy_code. Available from
https://github.com/PacktPublishing/ROS-Robotics-By-Example-Second-Edition/tree/master/Chapter08/
Turtlesim_joy_code [accessed 9 February 2020].

__________________________________________________________________________________________________________
Brief Description
__________________________________________________________________________________________________________

Here

"""
import rospy
from geometry_msgs.msg import Twist

def select_doorstep():

	# Create a publisher which can "talk" to Turtlesim and tell it to move
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
	 
	# Create a Twist message and add linear x and angular z values
	move_cmd = Twist()
	move_cmd.linear.x = 1.0
	move_cmd.angular.z = 1.0

	# Save current time and set publish rate at 10 Hz
	now = rospy.Time.now()
	rate = rospy.Rate(10)

	# For the next 6 seconds publish cmd_vel move commands to Turtlesim
	while rospy.Time.now() < now + rospy.Duration.from_sec(6):
		pub.publish(move_cmd)
		rate.sleep()

if __name__ == '__main__':
	try:
		select_doorstep()
	except rospy.ROSInterruptException:
		pass

# (Fairchild et al., 2017, 364-365)
# (Packt Publishing, 2017)