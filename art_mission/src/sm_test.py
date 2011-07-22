#!/usr/bin/env python
import roslib
roslib.load_manifest('art_mission')
import rospy
import smach
import smach_ros
import net_comm.srv
from subprocess import call
from std_msgs.msg import String

# Start up all your stuff here
class Start(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['ready', 'not_ready'])

	def execute(self, userdata):
		rospy.loginfo('Starting up. Waiting for services.')
		rospy.wait_for_service('/serializer/land')
		rospy.wait_for_service('/serializer/lift_off')
		rospy.wait_for_service('/serializer/set_hover_altitude')		
		return 'ready'

class LiftOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])
		self.attempt = 0
		self.max_attempts = 5
	def execute(self, userdata):
		try:
			lift = rospy.ServiceProxy('/serializer/lift_off', net_comm.srv.LiftOff)
			lift()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			if self.attempt >= self.max_attempts:
				return 'aborted'
			else:
				self.attempt += 1
				return 'preempted'
		
		return 'succeeded'

class LiftingOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])
		self.stub_altitude = 0
		self.desired_altitude = 80
			
	def execute(self, userdata):
		# Check altitude
		if self.stub_altitude < self.desired_altitude:
			self.stub_altitude += 1
			return 'preempted'
		return 'succeeded'
	
class Hover(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted'])
		self.started_at = rospy.Time.now()
		
	def execute(self, userdata):
		# Remain for however long
		if rospy.Time.now() < self.started_at + rospy.Duration(10):
			return 'preempted'
		return 'succeeded'

class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])
		self.attempt = 0
		self.max_attempts = 5
		
	def execute(self, userdata):
		try:
			land = rospy.ServiceProxy('/serializer/land', net_comm.srv.Land)
			land()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			if self.attempt >= self.max_attempts:
				return 'aborted'
			else:
				self.attempt += 1
				return 'preempted'
		
		return 'succeeded'

class Landing(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])
		self.desired_altitude = 0
		self.stub_altitude = 80
	def execute(self, userdata):
		# Check altitude
		if self.stub_altitude > self.desired_altitude:
			self.stub_altitude -= 1
			return 'preempted'
		return 'succeeded'


class Landed(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded'])
		
	def execute(self, userdata):
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