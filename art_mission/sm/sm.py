#!/usr/bin/env python
import roslib
roslib.load_manifest('art_mission')
import rospy
import smach
import smach_ros

from std_msgs.msg import String

import Initial, Enter, Hallway, Pickup, Return

# Start up all your stuff here
class Start(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
												outcomes = ['ready'])

	def execute(self, userdata):
		rospy.loginfo('Starting up')
		return 'ready'
		
def main():
	rospy.init_node('mission_control')
	
	sm_top = smach.StateMachine(outcomes=['end_mission_success'])
	sm_top.userdata.ready = 0
	
	with sm_top:
		smach.StateMachine.add('Start', Start(), 
													transitions={'ready':'Initial'})
		
		# Initial container
		sm_initial = smach.StateMachine(outcomes=['ready'])
		with sm_initial:
			smach.StateMachine.add('Initial.Post', Initial.Post(),
														transitions={'not_ready':'Initial.Post',
																				'ready':'Initial.LiftOff'})
			smach.StateMachine.add('Initial.LiftOff', Initial.LiftOff(),
														transitions={'lifting_off':'Initial.LiftOff',
																				'done':'ready'})
		smach.StateMachine.add('Initial', sm_initial, transitions={'ready':'Enter'})
		
		
		# Enter the office
		sm_enter = smach.StateMachine(outcomes=['ready'])
		with sm_enter:
			smach.StateMachine.add('Enter.Detect', Enter.Detect(),
														transitions={'found':'Enter.FlyIn',
																				'not_found':'Enter.Detect'})
			smach.StateMachine.add('Enter.FlyIn', Enter.FlyIn(),
														transitions={'flying':'Enter.FlyIn',
																				'done':'ready'})
		smach.StateMachine.add('Enter', sm_enter, transitions={'ready':'Hallway'})
		
		# Navigate the hallways
		sm_hallway = smach.StateMachine(outcomes=['ready'])
		with sm_hallway:
			smach.StateMachine.add('Hallway.Placeholder', Hallway.Placeholder(), transitions={'ready':'ready'})
		smach.StateMachine.add('Hallway', sm_hallway, transitions={'ready':'Pickup'})
		
		# Pickup the flash drive
		sm_pickup = smach.StateMachine(outcomes=['ready'])
		with sm_pickup:
			smach.StateMachine.add('Pickup.Placeholder', Pickup.Placeholder(), transitions={'ready':'ready'})
		smach.StateMachine.add('Pickup', sm_pickup, transitions={'ready':'Return'})
		
		# Return to base/judge
		sm_return = smach.StateMachine(outcomes=['ready'])
		with sm_return:
			smach.StateMachine.add('Return.Placeholder', Return.Placeholder(), transitions={'ready':'ready'})
		smach.StateMachine.add('Return', sm_return, transitions={'ready':'end_mission_success'})
					
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