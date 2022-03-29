#!/usr/bin/env python

# https://github.com/pirobot/rbx1/blob/indigo-devel/rbx1_nav/nodes/nav_test.py

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def goal_state():

		rospy.init_node('autonomy', anonymous=True)
		
		location = Pose(Point(12.543, 7.831, 0.000), Quaternion(0.000, 0.000, 0.223, 7.831))
		
		# Subscribe to the move_base action server
		move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server...")
		# Wait 60 seconds for the action server to become available
		move_base.wait_for_server(rospy.Duration(60))    
		rospy.loginfo("Connected to move base server")

		# Set up the next goal location
		goal = MoveBaseGoal()
		goal.target_pose.pose = location
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		
		# Let the user know where the robot is going next
		rospy.loginfo("Going to: ")
		
		# Start the robot toward the next location
		move_base.send_goal(goal)
		
		# Allow 5 minutes to get there
		finished_within_time = move_base.wait_for_result(rospy.Duration(300)) 
		
		# Check for success or failure
		if not finished_within_time:
			move_base.cancel_goal()
			rospy.loginfo("Timed out achieving goal")
		else:
			state = move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo("Goal succeeded!")
			else:
			  rospy.loginfo("Goal failed...")
		
if __name__ == '__main__':
	try:
		goal_state()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AMCL navigation test finished.")