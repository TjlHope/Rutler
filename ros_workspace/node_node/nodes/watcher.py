#!/usr/bin/env python

import roslib; roslib.load_manifest('rutler_interface')
import rospy
from std_msgs.msg import String

def logCommand(data):
    rospy.loginfo(rospy.get_name()+ " I heard %s",data.data)

def user_watcher():
    rospy.init_node('user_watcher', anonymous=True)
    rospy.Subscriber("user_input", String, logCommand)
    rospy.spin()

if __name__ == '__main__':
    try:
        user_watcher()
    except rospy.ROSInterruptException: pass
