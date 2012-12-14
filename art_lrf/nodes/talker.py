#!/usr/bin/env python
import roslib; roslib.load_manifest('art_lrf')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
def talker():
    pub = rospy.Publisher('chatter', String)
    pub2 = rospy.Publisher('number', Int32)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str1 = "hello world %s"%rospy.get_time()
        str2 = "hello world2 %s"%rospy.get_time()
        num1 = 3
        rospy.loginfo(str1)
        rospy.loginfo(num1)
        pub.publish(String(str1))
        pub2.publish(Int32(num1))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
