#!/usr/bin/env python

import roslib; roslib.load_manifest('rutler_interface')
import rospy
from std_msgs.msg import String

def looker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('looker')
    i = 0
    while not rospy.is_shutdown():
        if(i % 2): # even
            str = '{"mouth": "look"}'
        else:
            str = '{"mouth": "smile"}'
        i += 1
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        looker()
    except rospy.ROSInterruptException: pass
