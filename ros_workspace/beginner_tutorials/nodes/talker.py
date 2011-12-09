#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import String
#bum = 1
def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():## and x == 1:
    #if bum == 1:
	str = '{ "move": "301" }'
	rospy.loginfo(str)
	pub.publish(String(str))
	rospy.sleep(1.0)
	#bum = 2
if __name__ == '__main__':
    #bum = 1
    try:
        talker()
    except rospy.ROSInterruptException: pass

