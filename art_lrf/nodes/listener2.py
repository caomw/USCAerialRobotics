#!/usr/bin/env python
import roslib; roslib.load_manifest('art_lrf')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %d",data.data)

def listener():
    rospy.init_node('listener2', anonymous=False)
    rospy.Subscriber("number", Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
