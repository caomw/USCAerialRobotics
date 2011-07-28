import roslib
roslib.load_manifest('art_mission')
import rospy
from std_msgs.msg import String, Int16, Bool
from geometry_msgs.msg import Pose2D, Vector3

""" Publishes commands to the robot """
class beohawk():
	liftoff_altitude = 80
	landed_altitude = 20
	roll = 0.0
	pitch = 0.0
	yaw = 0.0
	alt = 0.0
	def __init__(self):
		self.log("Pilot initialized.")
		# Publishers
		self.arm_motors_topic = rospy.Publisher('/arduino/arm_motors', Bool)
		self.set_altitude_topic = rospy.Publisher('/arduino/set_altitude', Int16)
		self.set_waypoint_topic = rospy.Publisher('/arduino/set_waypoint', Pose2D)
		# Subscribers
		rospy.Subscriber("/arduino/altitude", Int16, self.get_altitude)	
		rospy.Subscriber("/arduino/rotation", Vector3, self.get_rotation)    
	
	def log(self, string):
		rospy.loginfo("[PILOT] %s"%string)
	
	
	""" Actions """
	def lift_off(self):
		self.log("Sending liftoff command")
		self.set_altitude_topic.publish(self.liftoff_altitude)
	
	def land(self):
		self.log("Initiating a controlled landing")
		self.set_altitude_topic.publish(0)
	
	def arm_motors(self):
		self.log("Arming motors")
		self.arm_motors_topic.publish(True)
		
	def disarm_motors(self):
		self.log("Disarming motors")
		self.arm_motors_topic.publish(False)
	
	""" Subscription Callbacks """
	def get_rotation(self, rotation):
		self.roll = rotation.x
		self.pitch = rotation.y
		self.yaw = rotation.z
		
	def get_altitude(self, altitude):
		self.alt = altitude.data
		
# Assign a global var
beohawk = beohawk();