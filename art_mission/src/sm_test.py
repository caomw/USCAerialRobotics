#!/usr/bin/env python
import roslib
roslib.load_manifest('art_mission')
import rospy
import smach
import smach_ros
from subprocess import call
from time import sleep
from lib_command import beohawk

# Start up all your stuff here
class Start(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['ready', 'not_ready'])

	def execute(self, userdata):
		rospy.loginfo('Starting up. Waiting for services.')
		beohawk.arm_motors()
		return 'ready'

class LiftOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])

	def execute(self, userdata):
		beohawk.lift_off()
		return 'succeeded'

class LiftingOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])
			
	def execute(self, userdata):
		# Check altitude
		if(beohawk.alt < beohawk.liftoff_altitude):
			return 'preempted'
		else:
			return 'succeeded'
	
class Hover(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted'])
		self.started_at = rospy.Time.now()
		
	def execute(self, userdata):
		# Remain for however long
		if rospy.Time.now() < self.started_at + rospy.Duration(1):
			return 'preempted'
		return 'succeeded'

class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])
		self.attempt = 0
		self.max_attempts = 5
		
	def execute(self, userdata):
		beohawk.land()
		return 'succeeded'

class Landing(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])
	def execute(self, userdata):
		# Check altitude
		if(beohawk.alt > beohawk.landed_altitude):
			return 'preempted'
		return 'succeeded'


class Landed(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded'])
		
	def execute(self, userdata):
		beohawk.disarm_motors()
		return 'succeeded'
		
		
def main():
	rospy.init_node('mission_control')
	
	sm_top = smach.StateMachine(outcomes=['end_mission_success','end_mission_fail'])
	sm_top.userdata.ready = 0
	
	with sm_top:
		smach.StateMachine.add('Start',      Start(),      transitions={'ready':'LiftOff', 'not_ready':'Start'})
		smach.StateMachine.add('LiftOff',    LiftOff(),    transitions={'succeeded':'LiftingOff','preempted':'LiftOff','aborted':'end_mission_fail'})
		smach.StateMachine.add('LiftingOff', LiftingOff(), transitions={'succeeded':'Hover','preempted':'LiftingOff','aborted':'end_mission_fail'})
		smach.StateMachine.add('Hover',      Hover(),      transitions={'succeeded':'Land', 'preempted':'Hover'})
		smach.StateMachine.add('Land',       Land(),       transitions={'succeeded':'Landing','preempted':'Land','aborted':'end_mission_fail'})
		smach.StateMachine.add('Landing',    Landing(),    transitions={'succeeded':'Landed','preempted':'Landing','aborted':'end_mission_fail'})
		smach.StateMachine.add('Landed',     Landed(),     transitions={'succeeded':'end_mission_success'})
		
		
					
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_top, '/MAIN')
	sis.start()

	# Execute the state machine
	outcome = sm_top.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()
  
#	

if __name__ == '__main__':
	main()