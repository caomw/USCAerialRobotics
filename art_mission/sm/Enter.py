#!/usr/bin/env python
import roslib
roslib.load_manifest('art_mission')
import rospy
import smach
import smach_ros

from std_msgs.msg import String

# Enter - Detect and enter the meter squared window into the office


# Look around for the window
class Detect(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['found', 'not_found'])
		self.found = 0
		
	def execute(self, userdata):
		rospy.loginfo('Looking for window')
		if self.found == 0:
			self.found += 1
			return 'not_found'
		else:
			return 'found'

# Enter the window
class FlyIn(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['flying', 'done'])
		self.count = 0
		
	def execute(self, userdata):
		rospy.loginfo('Flying in')
		if self.count < 3:
			self.count += 1
			return 'flying'
		else:
			return 'done'
