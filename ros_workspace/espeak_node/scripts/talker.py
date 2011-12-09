#!/usr/bin/env python

import sys

import roslib; roslib.load_manifest('espeak_node')
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('talk', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        string = sys.stdin.readline()
	pub.publish(String(string))
	rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
