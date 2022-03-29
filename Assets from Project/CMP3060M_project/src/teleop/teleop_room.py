#!/usr/bin/env python

"""
________________________________________________________________________________________________________
References
__________________________________________________________________________________________________________

ROS.org (2019) joy - ROS Wiki. Open Robotics. Available from 
http://wiki.ros.org/joy [accessed 9 February 2020].

Fairchild, C. and Harman, T.L. (2017) Ros Robotics By Example, 2nd Edition.
Birmingham, UK: Packt Publishing Ltd.

Packt Publishing (2017) Turtlesim_joy_code. PacktPublishing. Available from
https://github.com/PacktPublishing/ROS-Robotics-By-Example-Second-Edition/tree/master/Chapter08/
Turtlesim_joy_code [accessed 9 February 2020].

The Construct (2017) [ROS Q&A] 031 – How to read LaserScan data (ROS python). Alberto Ezquerro. 
Available from http://www.theconstructsim.com/read-laserscan-data/ [accessed 9 February 2020].

RISC Labs (2018) ROS Basics. Mohamed Abdelkader. Available from 
https://risc.readthedocs.io/1-ros-basics.html [accessed 9 February 2020].

risckaust (2018) ros-basics. Kuat Telegenov. Available from 
https://github.com/risckaust/risc-documentations/blob/master/src/ros-basics/mini_project.py 
[accessed 9 February 2020].

__________________________________________________________________________________________________________
Brief Description
__________________________________________________________________________________________________________

This script will allow for the teleoperation of the PWC and room selection within the household. It is 
possible to do this by using an Xbox 360 controller and the joy package. Specific buttons on the control
(Xbox power on/off, A, B, X and Y) call other scripts, when pressed. These scripts navigate to a goal,
which has been set in each of the assigned rooms. Autonomy is used to follow a planned path and reach the
desired location, entered by the user.

__________________________________________________________________________________________________________
1. Imported Clients, Packages and Scripts
__________________________________________________________________________________________________________
"""

import rospy # The rospy client API.

from geometry_msgs.msg import Twist # The ROS Message Type, Twist, is called from the geometry_msgs pkg.
from sensor_msgs.msg import Joy # The ROS Message Type, Joy, is called from the sensor_msgs pkg.

# Laser Scan: Could also collect some data as to the average object distance. Any collisions etc.

from room_select_bathroom import select_bathroom 		# [A]
from room_select_bedroom import select_bedroom 			# [B]
from room_select_livingroom import select_livingroom 	# [X]
from room_select_kitchen import select_kitchen 			# [Y]
from room_select_doorstep import select_doorstep 		# [Power/Xbox Button]

# ^^^^ These will all need to be launched from roslaunch

"""
__________________________________________________________________________________________________________
2. Assigned Velocities
__________________________________________________________________________________________________________

This sets the cmd_vel (command velocity) to:
	- linear velocity (Measured in Metres) = travels/moves forward at 40 centimetres per second 
	- angular velocity (Measured in Radians) = rotates 0.5 radians per second (approx 30 degrees p/s)

Note: The maximum angular velociy for a TB2 is 3.1 radians (approx) or 180 degrees p/s.
	  The PWC rotates at a speed of ....research

"""

assign_vel = [0.4, 0.5]

"""
__________________________________________________________________________________________________________
3. Function: Subscribe to User Input
__________________________________________________________________________________________________________
"""

def input_listener():

	"""
	This is intiates the node
	ROS assigns the uniques name, teleop_select, by using anonymous=True.
	This will allow multiple nodes to run easily.
	"""
	rospy.init_node("teleop_room", anonymous=True)

	"""
	Joy Topic subcribes to the topic Twist. 
	Joy will not receive cmd_vel messages.
	This callback treats new messages as the first arguement.
	This stream of communication will allow for constant control updates. 
	"""
	
	rospy.Subscriber("joy", Joy, input_callback, queue_size=1)
	
	# sub_scan = rospy.Subscriber('scan', LaserScan, nearing_obj)
	

	# Spin allows the script to stay active until the node stops.
	rospy.spin()

"""
__________________________________________________________________________________________________________
4. Function: Publish to User Input
__________________________________________________________________________________________________________
"""

def input_callback(left_stick):

	# The topic cmd_vel will publish diretly to the diffrential drive robot.
	pub_tele_vq = rospy.Publisher("cmd_vel", Twist, queue_size=1)

	# Assigned a new variable name for 'Twist'. being 'tele_vq' (Teleop Vector Quantity)
	tele_vq = Twist()

	"""
	How User Input is Received:
	___________________________

	The attribute .axes is imported with the Joy package. This refers to the axis of the xbox controller.
	There is an array of 8 variables, which include the joysticks, triggers and D-Pad/Cross keys of the
	controller. For the purpose of this study, the left thumstick will be used. The left/right movement
	of the left stick is position [0] of the array and up/down motion is position [1]. 
	
	(ROS.org, 2019)

	The code below takes the input of the user and multiplies the rate by an assigned value. This allows
	the PWC to move at very specific speeds using the Twist topic. Twist is re-assigned to tele_vq and 
	allows it to access the linear and angular attributes. The linear velocity commands move the robot 
	along x-axis within 3D space. While angular velocity commands are processed along the z-axis, as the 
	robot must simply rotate and not travel in any direction.  
	""" 
	tele_vq.linear.x = left_stick.axes[1] * assign_vel[0]
	tele_vq.angular.z = left_stick.axes[0] * assign_vel[1] 

	"""
	The below, loginfo attribute from the rospy pkg, will publish the outputs for linear and angular 
	velocity. As an example:
	
	If the PWC is traveling forwards for 3 seconds the terminal will display: 
		
		"tele_vq.linear: 0.8 ; angular 0.0"

	The same can be applied for a rotation for 2 seconds, which would display:

		"tele_vq.linear: 0.0 ; angular 1.0"
	""" 
	rospy.loginfo("Current Speed: linear %f | angular %f", tele_vq.linear.x, tele_vq.angular.z)
	rospy.loginfo("")

"""
__________________________________________________________________________________________________________
5. Xbox Buttons Assigned to room_select_[room].py scripts
__________________________________________________________________________________________________________

	
	
	
	
	

	# (ROS.org, 2019)

"""
	

	# green / A button on Xbox controller
	if data.buttons[0] == 1:
		select_bathroom()
		rospy.loginfo("Moving to: Bathroom")
		rospy.loginfo("") # Stops constant input stream of msgs

	# red / B button on Xbox controller
	if data.buttons[1] == 1: 
		select_bedroom()
		rospy.loginfo("Moving to: Bedroom")
		rospy.loginfo("")

	# blue / X button on Xbox controller
	if data.buttons[2] == 1: 
		select_livingroom()
		rospy.loginfo("Moving to: Living Room")
		rospy.loginfo("")

	# yellow / Y button on Xbox controller
	if data.buttons[3] == 1:        
		select_kitchen()
		rospy.loginfo("Moving to: Kitchen")
		rospy.loginfo("") 

	# power button / xbox Logo on Xbox controller	
	if data.buttons[8] == 1:       
		select_doorstep()
		rospy.loginfo("Moving to: Doorstep")
		rospy.loginfo("")


	# This publishes the float, 'tele_vq' (user input), to the cmd_vel topic.
	pub_tele_vq.publish(v)


"""
__________________________________________________________________________________________________________
6. Basic Object Detection
__________________________________________________________________________________________________________


def nearing_obj(area_scan):

	if area_scan.ranges[0] < 0.9:
		assign_vel[1] / 2  
		rospy.loginfo('AMBER: Less than 1m from object...')
		rospy.loginfo("")
		#  add controller rumble -- weak

	if area_scan.ranges[639] < 0.9:
		rospy.loginfo('AMBER: Less than 1m from object...')
		rospy.loginfo("")
		#  add controller rumble -- weak

	if area_scan.ranges[0] >= 1 and area_scan.ranges[0] < 2:
		rospy.loginfo('GREEN: approaching obstacle...')
		rospy.loginfo("")

	if area_scan.ranges[639] >= 1 and area_scan.ranges[639] < 2:
		rospy.loginfo('GREEN: approaching obstacle...')
		rospy.loginfo("")

	# Just have strong rumble on impact. Any collision can result in 'RED: COLLISION!!!'

	# (The Construct, 2017)
	# (RISC Labs, 2018)
	# (risckaust, 2018) 

"""

"""
__________________________________________________________________________________________________________
6. Execution
__________________________________________________________________________________________________________
"""

if __name__ == '__main__':
	# This loop allows for the joy_listener to continuously search for cmd_vel msgs 
	# This cmd_vel msgs will then be called back to the joy node. 
	try:
		input_listener()
	# This will continue until CTRL+C is pressed within the terminal
	except rospy.ROSInterruptException:
		pass

# (Fairchild et al., 2017, 362-364)
# (Packt Publishing, 2017)
