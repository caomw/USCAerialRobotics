#!/usr/bin/env python
import roslib
roslib.load_manifest('art_mission')
import rospy
import smach
import smach_ros

from std_msgs.msg import String

# Initial - Initialize ROS nodes, fly to fixed altitude


# Power on self test
class Post(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['not_ready', 'ready'])
		self.POST = 0
		
	def execute(self, userdata):
		rospy.loginfo('Testing nodes')
		if self.POST == 0:
			return 'ready'
		else:
			return 'not_ready'
			
# Fly to a fixed altitude
class LiftOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['lifting_off', 'done'])
		self.Altitude = 0
				
	def execute(self, userdata):
		rospy.loginfo('Lifting off')
		if self.Altitude == 15:
			return 'done'
		else:
			self.Altitude += 1
			return 'lifting_off'