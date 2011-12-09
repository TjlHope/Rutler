#!/usr/bin/env python

import sys

import roslib; roslib.load_manifest('simple_navigation_goals')
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('move_rutler', String)
    rospy.init_node('location_talker')
    while not rospy.is_shutdown():
        string = sys.stdin.readline()
        if string.lower().strip() == 'refresh':
            pub = rospy.Publisher('move_rutler', String)
        elif string.split()[0].strip() == 'read':
            with open(string.split()[1]) as fl:
                for line in fl.readlines():
                    pub.publish(String(line))
                    rospy.sleep(1.0)
        else:
            pub.publish(String(string))
            rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
