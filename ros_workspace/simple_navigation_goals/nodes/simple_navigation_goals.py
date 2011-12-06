#!/usr/bin/env python

import roslib
roslib.load_manifest('simple_navigation_goals')

import rospy
from std_msgs.msg import String
import actionlib
import move_base_msgs.msg

try:
    from misc.input import csv2dict
except ImportError:
    from python_misc.input import csv2dict


goal = None
client = None
#dest = Coord(0, 0, 1)
#x = 0
#y = 0
z = p = q = r = 0
#w = 1
room = "310"
chamber = "302"


def read_csv(file_name):
    # TODO: get from Tom
    return csv2dict(file_name)


def go_to_dest(dest):
    global client, goal         # globals are dirty but f*** it
    if client:         # do nothing if not inititalised
        
        if (goal.goal.target_pose.pose.position.x,
                goal.goal.target_pose.pose.position.y,
                goal.goal.target_pose.pose.orientation.w) != dest:
            rospy.loginfo("Sending goal: %s", dest)
            # cancel current goal
            client.cancel_goal()
            # redo goal
            (goal.goal.target_pose.pose.position.x,
                goal.goal.target_pose.pose.position.y,
                goal.goal.target_pose.pose.orientation.w) = dest
            goal.goal.target_pose.header.stamp = rospy.Time.now()
            # send new goal
            client.send_goal(goal.goal)
        else:
            rospy.loginfo("Same Goal, do nothing.")

        #print 'state', client.get_state()
        #print 'result', client.get_result()


def callback(data):
    rospy.loginfo("%s I heard %s", rospy.get_name(), data.data)
    if data.data == room:
        global x, y, w
        rospy.loginfo("WOOOOO")
        dest = Coord(30, 20, 1)
        #x = 30
        #y = 20
        #w = 1.0
    elif data.data == chamber:
	global x, y, w
	rospy.loginfo("W00000")
        dest = Coord(31, 19, 1)
	#x = 34
	#y = 19
	#w = 1.0
    else:
        return
    rospy.loginfo("x: %d; y: %d;", dest.x, dest.y)
    go_to_dest(dest)


def listener():
#    rospy.Subscriber("chatter", String, callback)
    rospy.init_node('simple_navigation_goals', anonymous=True)

    global client
    client = actionlib.SimpleActionClient('move_base',
            move_base_msgs.msg.MoveBaseAction)

    while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
        rospy.loginfo("Waiting for the move_base action server to come up")

    global goal
    goal = move_base_msgs.msg.MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = "map"
    goal.goal.target_pose.header.stamp = rospy.Time.now()

    # Subscribe to topic
    rospy.Subscriber("chatter", String, callback)
    rospy.loginfo("%s", String)

    #global goal
    #goal = move_base_msgs.msg.MoveBaseActionGoal()
    
#    rospy.Subscriber("chatter", String, callback)

    #goal.goal.target_pose.header.frame_id = "map"
    #goal.goal.target_pose.header.stamp = rospy.Time.now()
    
    #goal.goal.target_pose.pose.position.x = x
    #goal.goal.target_pose.pose.position.y = y
    #goal.goal.target_pose.pose.orientation.w = w

    #rospy.loginfo("Sending goal: %d", x)
    
    #client.send_goal(goal.goal)

    #client.wait_for_result()

    #if client.get_state() == actionlib.SimpleGoalState.DONE:
        #rospy.loginfo("Hooray, the base moved 1 meter forward")
    #else:
        #rospy.loginfo("The base failed to move forward 1 meter for some reason")

    # END
    #rospy.spin()
    while not rospy.is_shutdown():## and x == 1:
        if client:
            rospy.loginfo('state: %s', client.get_state())
            rospy.loginfo('result: %s', client.get_result())
            rospy.sleep(5.0)


if __name__ == '__main__':
 
    #rospy.Subscriber("chatter", String, callback)
    #rospy.loginfo("%s", String)
    #rospy.sleep(3.0)
    try:
        listener()
    except KeyboardInterrupt:
        print "Caught Interupt, cancelling goals..."
        client.cancel_all_goals()
    #rospy.spin()
