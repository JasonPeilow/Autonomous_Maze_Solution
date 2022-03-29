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

# (ROS.org, 2018)

# TODO -- which cmd_vel does this use? Mobile velocties or something?
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# (ROS.org, 2019)

"""
__________________________________________________________________________________________________________
2. Pose and Orientation
__________________________________________________________________________________________________________

	A. ( 0.0,  4.5)    E. (  0.0, -4.5)    K. (-2.25, -4.5)    R. ( 2.25,  4.5)
	B. ( 4.5,  4.5)    F. ( -4.5, -4.5)    L. (  0.0,-2.25)    S. (  0.0, 2.25)
	C. ( 4.5,  0.0)    G. ( -4.5,  0.0)    M. ( 2.25, -4.5)    T. (-2.25,  4.5)
	D. ( 4.5, -4.5)    H. ( -4.5,  4.5)    N. (  4.5,-2.25)    U. ( -4.5, 2.25)
	E. ( 0.0, -4.5)    I. (-2.25,  0.0)    P. ( 2.25,  0.0)
	F. (-4.5, -4.5)    J. (  4.5,-2.25)    Q. (  4.5, 2.25)

"""

GoalPoints = [[(0.0, 4.5, 0.0), (0.0, 0.0, 0.0, 1.0)] ,   # START: Begin Layer A
			[(4.5, 4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,     # B
			[(4.5, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # C
			[(0.0, 4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # A
			[(4.5, 4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # B
			[(0.0, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # ORIGIN
			[(4.5, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # C
			[(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # D
			[(-4.5, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # E
			[(4.5, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # C
			[(4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # D
			[(0.0, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # ORIGIN
			[(-4.5, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # G
			[(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # F
			[(0.0, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # E
			[(0.0, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # ORIGIN
			[(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # F
			[(0.0, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # E
			[(-4.5, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # G
			[(-4.5, 4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # H
			[(0.0, 4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # A
			[(-4.5, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # G
			[(-4.5, 4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # End of Layer A
			[(0.0, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,	  # ORIGIN: Switch to Layer B
			[(-2.25, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # I
			[(4.5, -2.25, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # J
			[(-2.25, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,  # K
			[(0.0, -2.25, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # L
			[(2.25, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # M
			[(4.5, -2.25, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # N
			[(2.25, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # P
			[(4.5, 2.25, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # Q
			[(2.25, 4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # R
			[(0.0, 2.25, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # S
			[(-2.25, 4.5, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # T
			[(-4.5, 2.25, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # U
			[(-2.25, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # I
			[(0.0, 2.25, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # S
			[(2.25, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,    # P
			[(0.0, -2.25, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # L
			[(-2.25, 0.0, 0.0), (0.0, 0.0, 0.7, 0.7)] ,   # I
			[(-4.5, -4.5, 0.0), (0.0, 0.0, 0.7, 0.7)]]    # ORIGIN: Repeat Layer A

"""
__________________________________________________________________________________________________________
3. Function: Cycle Through Goal States
__________________________________________________________________________________________________________
"""

def auto_pure(pose):

	auto_pure = MoveBaseGoal()
	auto_pure.target_pose.header.frame_id = 'map'
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

	rospy.init_node('Think_and_Act')

	#TODO does this have some kind of log output? add some? if not?
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