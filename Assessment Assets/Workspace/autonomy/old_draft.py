#!/usr/bin/env python

import array
import math
from math import pi, radians
from pprint import pformat

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
from tf import transformations

import numpy as np


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


class sense(Jareth_the_Goblin_King):

	def scan_perimeter(self, detect):
		# pass in forward kinematics
		# No forward kinemtics is bullshit. Needs command line or some fidly shit
		if detect.ranges[320] > 1 or math.isnan(detect.ranges[320]):
			self.forward_vel = True

		if detect.ranges[0] < 0.3:
			#[0:160] would be a cone for the left
			self.left_vel = True

		if detect.ranges[639] < 0.3:
			self.right_vel = True

		#if detect.ranges[320] < 0.3:
			#180 spin1
			#pass


		# output a new (v, a)
		# this will be used by the control methods the 'act' class

		self.pub_cmd_vel.publish(self.t)

	def left_door(self, open_left):
		if open_left.range[0] > 2:
			self.pass_left


	def right_door(self, open_right):
		if open_right.range[639] > 2:
			self.pass_right


	def capture_perimeter(self, shot):
		pass


class think(Jareth_the_Goblin_King):


	# Exactly what the name states. A functional block to open windows to view images.
	def open_windows(self, event): # wtf is the event? check ws4 opt
		try:
			# Just some naming conventions for each window. "WINDOW_NORMAL" makes the window more versatile (snaps to sides, enlarges etc)
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
		Below, blue (rgb) is used to determine the value for blue (hsv):
		 --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  -- -

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


	def compare_images(self, recon):
		# Tensor flow shiz -- NO
		pass


class act(Jareth_the_Goblin_King):

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