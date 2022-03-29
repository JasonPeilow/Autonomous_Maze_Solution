#!/usr/bin/env python

"""
__________________________________________________________________________________________________________
Brief Description
__________________________________________________________________________________________________________
Here
# (Quigley et al., 2015)
__________________________________________________________________________________________________________
1. Imported Clients and Packages
__________________________________________________________________________________________________________
"""
import rospy
import actionlib
import cv2

# (ROS.org, 2018)

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# (ROS.org, 2019)
import array
import math
from math import pi, radians
from pprint import pformat

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
from tf import transformations

import numpy as np

"""
__________________________________________________________________________________________________________
2. Pose and Orientation
__________________________________________________________________________________________________________
"""

GoalPoints = [[(4.5, 4.5, 0.0), (0.0, 0.0, 0.0, 1.0)] , 	# Origin(0) to A
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# A to D
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# D to A
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# A to B
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# B to 0
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# 0 to B
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# B to C
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# C to 0
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# 0 to C
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# C to D
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# D to 0
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# 0 to D
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# D to E
    		 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# E to A
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# A to E
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# E to F
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# F to A
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# A to F
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# F to G
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# G to A
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# A to G
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# G to B
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# B to G
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# G to H
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# H to B
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# B to H
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# H to I
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# I to B
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# B to I
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# I to C
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# C to I
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# I to J
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# J to C
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# C to J
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# J to K
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# K to C
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# C to K
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# K to D
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# D to K
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,		# K to L
			 [(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	# L to D
			 [(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)]] 		# D to L

"""
__________________________________________________________________________________________________________
2. Pose and Orientation
__________________________________________________________________________________________________________
"""

class Jareth_the_Goblin_King():

	def __init__(self):
		self.node_name = "Mah_bay_BEH"
		rospy.init_node(self.node_name)

		self.cv_window_name = self.node_name # is this actually needed?
		self.bridge = CvBridge()

		self.hz = rospy.Rate(10)
		self.t = Twist

		# REF TB2 manual
		self.wheel_radius = 0.038
		self.robot_radius = 0.177

		self.forward_vel = False
		self.left_vel = False
		self.right_vel = False

		self.pass_left = False
		self.pass_right = False

		self.sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
		self.sub_odoom = rospy.Subscriber("/odom", Odometry, self.callback)
		self.sub_wheel = rospy.Subscriber("/moving", Float32, self.turn_left)

		self.pub_cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)


		rospy.Timer(rospy.Duration(0.03), self.open_windows)

		rospy.spin()

		"""
		def forward_kinematics(self, w_l, w_r):
			w_l = 1.0
			w_r = 0.0

			c_l = self.wheel_radius * w_l
			c_r = self.wheel_radius * w_r

			v = (c_l + c_r) / 2
			a = (c_l - c_r) / (2 * self.robot_radius)

			return (v, a)
		"""

	"""
	__________________________________________________________________________________________________________
	3. Pose and Orientation
	__________________________________________________________________________________________________________
	"""
	def odom_orientation(self, q):
		# (y, p, r) yaw, pitch and roll
		# (q.w, q.y, q.z, q.w) our qauternions/orientation
		#
		y, p, r = transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
		# yaw * 180 (degrees) divided by 3.141592... (pi)
		return y * 180 / pi


	def callback(self, data):
		print("odom pose: \n" + pformat(data.pose.pose))
		angle = self.odom_orientation(data.pose.pose.orientation)
		print("angle = %f" % angle)

	"""
	__________________________________________________________________________________________________________
	3. Pose and Orientation
	__________________________________________________________________________________________________________
	"""
	# Exactly what the name states. A functional block to open windows to view images.
	def open_windows(self, event): # wtf is the event? check ws4 opt
		try:
			# Just some naming conventions for each window. "WINDOW_NORMAL"
   			# makes the window more versatile (snaps to sides, enlarges etc)
			cv2.namedWindow("Cammy", cv2.WINDOW_NORMAL)
			cv2.namedWindow("Slice", cv2.WINDOW_NORMAL)

			# "Cammy" is just the default RGB (BGR) camera view.
			cv2.imshow("Cammy", self.cam_view)
			# "Slice" displays the segmented colour (HSV) set within 'color_slice'.
			cv2.imshow("Slice", self.processed_image)

			# This is not really needed, but meh. Just something to give a bit of pause.
			cv2.waitKey(3)
		except:
			pass # bad exception handling. It works. Screw it.


	def image_callback(self, info):
		try:
			# calling the class variable, bridge [CvBridge()]
			# This takes the ROS Image messages and convert them into OpenCV format.
			# "bgr8" is and 8-bit RGB Image.
			# The reason it is backwards (RGB) is because the image is perceived by the camera as inverse matrices. I think?
			self.cam_view = self.bridge.imgmsg_to_cv2(info, "bgr8")
		except CvBridgeError ("e"): # try and catch called from Cv2
			print ("e") # print whatever did/did not happen (error)
			pass #...

		cam_view = np.array(self.cam_view, dtype = np.uint8)
		self.processed_image = self.color_slice(cam_view)


	def colour_config(self, rgb):

		"""
		__________________________________________________________________________________________________________
		3. Pose and Orientation
		__________________________________________________________________________________________________________

		When an RGB cube is viewed in the chromacity plane/as a hexacone,
		the colour values for hue become clear. It can be seen that each
		of the 6 colours is designated to a specific radiant/segment
		(red = 0, magenta = 60 blue = 120, cyan = 180, green = 240 and yellow = 360).
		This confirms that the caluclated values for R, G and B are correct.

		https://en.wikipedia.org/wiki/HSL_and_HSV

		This is the only value for blue, green and red. The min is 0.
		Therefore, the max cannot be subtracted from. The is no range of
		B, G or R. So there is not chroma either. The tone of B, G and R
		is defined by saturation and value.

		"""
		rgb_upper = [255, 255, 255]
		rgb_lower = [255, 255, 255]

		print(rgb_upper)
		print(rgb_lower)

		print(rgb_upper[0])
		print(rgb_upper[1])
		print(rgb_upper[2])

		print(rgb_upper[0]) + (rgb_lower[1]) # so it is calculable...idiots

		red_hue = (60 * (rgb_upper[1]-rgb_upper[2]) + 360) % 360
		print(red_hue)

		green_hue = (60 * (rgb_upper[2]-rgb_upper[0]) + 120) % 360
		print(green_hue)

		blue_hue = (60 * (rgb_upper[0]-rgb_upper[1]) + 240) % 360
		print(blue_hue)

		hue_array = []

		hue_array.append(blue_hue)
		hue_array.append(green_hue)
		hue_array.append(red_hue)

		print(hue_array)

		print(hue_array[0])
		print(hue_array[1])
		print(hue_array[2])


		#return (blue_hue, green_hue, red_hue)
		# array becomes [240, 120, 0]

		# it works
		"""
		__________________________________________________________________________________________________________
		3. Pose and Orientation
		__________________________________________________________________________________________________________

		Below, blue (rgb) is used to determine the value for blue (hsv):

		4 Rad * 180/pi = 229.183 ...
		2.7462 is the difference between a 60 degrees and 57.2358 (an exact Rad)
		compensate for 2.7462 degrees * 4 Rad = 11.0568 degrees
		229.183 + 11.0568 = 240.2938 (or 240 degrees)

		240 is the hue value of blue. red and green are calculated simiarly.
		"""

		# camera feed may be set to HSV, but the values must be set in HSV also.
		hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)

		# This really needs updating
		lower = [hue_array[0], 50, 25]
		upper = [hue_array[0], 100, 100]

		mask = cv2.inRange(hsv, lower, upper) # create a mask based on those boundaries (view shows hsv colours in range)
		masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask = mask) # mask the camera view to segment out the colour green

		h, w, d = cam_view.shape # height, width, depth of the camera display (for the mean stuff)
		M = cv2.moments(mask)

		# I will explain this properly tomorrow, but you probably get it
		if M['m00'] > 0:
			# centroids, x and y
			# checking moments received by the camera for the specific colour
			# this will be in the range of (REF PRwR book)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(cam_view, (cx, cy), 20, (255, 0, 0), -1)
			err = cx - w/2
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100

			self.cmd_vel_pub.publish(self.twist)

		return masked

	"""
	__________________________________________________________________________________________________________
	3. Pose and Orientation
	__________________________________________________________________________________________________________
	"""

	def onward(self, pub_for0ward):
		# pass sensor arguement (ranges etc)
		if self.forward_vel == True:

			pub_forward = Twist()
			pub_forward.linear.x = 0.5
			pub_forward.angular.z = 0.0

			self.pub_cmd_vel.publish(pub_forward)
		else:
			pass

	def turn_left(self, pub_left):
		if self.left_vel == True:

			pub_left = Twist()
			pub_left.linear.x = 0.0
			pub_left.angular.z = 0.2

			self.pub_cmd_vel.publish(pub_left)
		else:
			pass

	def turn_right(self, pub_right):
		if self.right_vel == True:

			pub_right = Twist()
			pub_right.linear.x = 0.0
			pub_right.angular.z = -0.2

			self.pub_cmd_vel.publish(pub_right)
		else:
			pass

"""
__________________________________________________________________________________________________________
3. Function: Cycle Through Goal States
__________________________________________________________________________________________________________
"""
def auto_pure(pose):

	auto_pure = MoveBaseGoal()
	auto_pure.target_pose.header.frame_id = 'base_link'
	auto_pure.target_pose.pose.position.x = pose[0][0]
	auto_pure.target_pose.pose.position.y = pose[0][1]
	auto_pure.target_pose.pose.position.z = pose[0][2]
	auto_pure.target_pose.pose.orientation.x = pose[1][0]
	auto_pure.target_pose.pose.orientation.y = pose[1][1]
	auto_pure.target_pose.pose.orientation.z = pose[1][2]
	auto_pure.target_pose.pose.orientation.w = pose[1][3]

	return auto_pure

	rospy.spin()
"""


__________________________________________________________________________________________________________
4. Execution
__________________________________________________________________________________________________________
"""

if __name__ == '__main__':

	rospy.init_node('find_dat_goal')

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	while True:
		for pose in GoalPoints:
			goal = auto_pure(pose)
			client.send_goal(goal)
			client.wait_for_result()


# (Fairchild et al., 2017, 189-191)
# (Packt Publishing, 2017)

"""
__________________________________________________________________________________________________________
References
__________________________________________________________________________________________________________
ROS.org (2018) actionlib - ROS Wiki. Open Robotics. Available from
http://wiki.ros.org/actionlib [accessed 9 February 2020].

ROS.org (2019) move_base_msgs - ROS Wiki. Open Robotics. Available from
http://wiki.ros.org/move_base_msgs [accessed 9 February 2020].

Quigley, M., Gerkey, B. and Smart, W.D. (2015) Programming Robots with ROS.
Sebastopol, USA: O'Reilly Media, Inc.

Fairchild, C. and Harman, T.L. (2017) Ros Robotics By Example, 2nd Edition.
Birmingham, UK: Packt Publishing Ltd.

Packt Publishing (2017) Chapter04. Github. Available from
https://github.com/PacktPublishing/ROS-Robotics-By-Example-Second-Edition/tree/master/Chapter04
[accessed 9 February 2020].
"""