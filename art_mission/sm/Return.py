#!/usr/bin/env python
import roslib
roslib.load_manifest('art_mission')
import rospy
import smach
import smach_ros

from std_msgs.msg import String

# Return - Haul ass back home



# Placeholder
class Placeholder(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['ready'])
		
	def execute(self, userdata):
		rospy.loginfo('Placeholder')
		return 'ready'