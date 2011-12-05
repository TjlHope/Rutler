#!/usr/bin/env python

import roslib; roslib.load_manifest('rutler_interface')
import rospy
from std_msgs.msg import String


def looker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('looker')
    commands = ['look', 'query', 'acknowledge', 'happy', 'sad', 'open', 'wink', 
    'laughing'];
    while not rospy.is_shutdown():
        for item in commands:
            string = '{"face":"%s"}' % item
            rospy.loginfo(string)
            pub.publish(String(string))
            rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        looker()
    except rospy.ROSInterruptException: pass
