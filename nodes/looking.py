#!/usr/bin/env python

import roslib; roslib.load_manifest('rutler_interface')
import rospy
from std_msgs.msg import String


def looker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('looker')
    commands = {
        "look": '{"mouth": "look"}',
        "query": '{"mouth": "small"}',
        "acknowledge": '{"mouth": "smileopen"}',
        "happy": '{"mouth": "smile"}',
        "sad": '{"mouth": "sad"}',
        "open": '{"mouth": "open"}'
    }
    while not rospy.is_shutdown():
        for item in commands:
            rospy.loginfo(commands[item])
            pub.publish(String(commands[item]))
            rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        looker()
    except rospy.ROSInterruptException: pass
