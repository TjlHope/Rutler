#!/usr/bin/env python

import sys 
import os
import json
from collections import namedtuple

import roslib
roslib.load_manifest('simple_navigation_goals')

import rospy
from std_msgs.msg import String

import actionlib
import move_base_msgs.msg
from tf import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped

try:
    from misc.input import csv2dict
except ImportError:
    from python_misc.input import csv2dict


goal_state = [
               'PENDING',
               'ACTIVE',
               'PREEMPTED',
               'SUCCEEDED',
               'ABORTED',
               'REJECTED',
               'PREEMPTING',
               'RECALLING',
               'RECALLED',
               'LOST'
              ]

goal = None
client = None

# receive from interface
# "{ "action": "move", "value": "301" }"
# "{ "action": "start", "value": "6" }"
### "{ "action": { "start" : "6" } }" 
### "{ "action": { "move" : "301" } }" 
### "{ "move": "301" }"


def get_coords(filename, roomnum): #added by seb
	csvreader = csv.DictReader(open(filename,'rb'))
	# we will ignore the headers, because the format will be fixed to
	# roomnum,w,x,y
	w, x, y = (0, 0, 0)	
	for row in csvreader:
		if int(row['room']) == roomnum:
			w, x, y = float(row['w']), float(row['x']), float(row['y'])
			break
	return (x, y, w)


def set_origin(origin):
    rospy.loginfo("Setting origin: %s", origin)
    p = PoseWithCovarianceStamped()
    p.header.frame_id = "/map"
    p.pose.pose.position.x = origin.x
    p.pose.pose.position.y = origin.y
    (p.pose.pose.orientation.x, 
     p.pose.pose.orientation.y, 
     p.pose.pose.orientation.z, 
     p.pose.pose.orientation.w) = \
             transformations.quaternion_from_euler(0, 0, origin.r)
    #p.pose.covariance[6*0+0] = 0.5 * 0.5
    p.pose.covariance[0] = 0.25
    #p.pose.covariance[6*1+1] = 0.5 * 0.5
    p.pose.covariance[7] = 0.25
    #p.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0
    p.pose.covariance[21] = 0.06853891945200942
    rospy.loginfo("Publish pose: %s", p)
    pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped)
    pub.publish(p)


def init_goal(dest):
    "dest = (x, y, r)"
    goal = move_base_msgs.msg.MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = "/map"
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    goal.goal.target_pose.pose.position.x = dest.x
    goal.goal.target_pose.pose.position.y = dest.y
    (goal.goal.target_pose.pose.orientation.x, 
     goal.goal.target_pose.pose.orientation.y, 
     goal.goal.target_pose.pose.orientation.z, 
     goal.goal.target_pose.pose.orientation.w) = \
             transformations.quaternion_from_euler(0, 0, dest.r)
    return goal


def go_to_dest(dest):
    global client, goal         # globals are dirty but f*** it
    if client:         # do nothing if not inititalised
        if not (goal.goal.target_pose.pose.position.x == dest.x and
                goal.goal.target_pose.pose.position.y == dest.y and
                transformations.euler_from_quaternion(
                    goal.goal.target_pose.pose.orientation)[2] == dest.r):
            # cancel current goal
            client.cancel_goal()
            # redo goal
            goal = init_goal(dest)
            # send new goal
            rospy.loginfo("Sending goal: %s", goal)
            client.send_goal(goal.goal)
        else:
            rospy.loginfo("Same Goal, do nothing.")

        #print 'state', client.get_state()
        #print 'result', client.get_result()


def stop():
    global client       # globals are dirty but f*** it
    if client:          # do nothing if not inititalised
        rospy.loginfo("Resuming goal: %s", goal)
        # cancel current goal
        client.cancel_all_goals()


def resume():
    global client, goal         # globals are dirty but f*** it
    if client:          # do nothing if not inititalised
        rospy.loginfo("cancelling all goals.")
        # cancel current goal
        client.send_goal(goal.goal)


def callback(data):
    global storeGoal, currentFloor
    rospy.loginfo("%s I heard %s", rospy.get_name(), data.data)
    try:
    	location = json.loads(data.data)
        if location['action'] == 'stop':
            stop()
        if location['action'] == 'resume':
            resume()
        else:
            try:
                room = rooms_dict[location['value']]
            except KeyError:
                rospy.logerr("Room %s not in rooms data.", data.data)
                return False
            dest = room._make([float(d) for d in room])
            #print dest
            if location['action'] == 'start':
                # load intial coordinates
                set_origin(dest)
            elif location['action'] == 'move':
                go_to_dest(dest)
                #rospy.loginfo("x: %d; y: %d;", dest.x, dest.y)
            else:
                rospy.logerr("Someone fucked up: %s", location)
    except ValueError:
    	rospy.logerr("Data.data is not a json string. It is %s.", data.data)


def listener():
#    rospy.Subscriber("chatter", String, callback)
    rospy.init_node('simple_navigation_goals', anonymous=True)

    global client
    client = actionlib.SimpleActionClient('move_base',
            move_base_msgs.msg.MoveBaseAction)

    try:
        while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("Waiting for the move_base action server to come up")
    except KeyboardInterrupt:
        print "Caught SIGINT when waiting for move_base action server."
        return False

    global goal 
    Dest = namedtuple('Dest', 'x y r')
    goal = init_goal(Dest(0, 0, 0))

    # Subscribe to topic
    rospy.Subscriber("move_rutler", String, callback)
    rospy.loginfo("%s", String)

    status = rospy.Publisher("rutler_status", String)

    # END
    #rospy.spin()
    suc_sent = False
    while not rospy.is_shutdown():##and x == 1:
        if client:
            if client.get_state == 3:
                if suc_sent:
                    continue
                else:
                    suc_sent = True
            else:
                suc_sent = False
            rospy.loginfo('state: %s', goal_state[client.get_state()])
            status.publish(
            '{"status": "'+goal_state[client.get_state()]+'"}'
            )
            rospy.loginfo('result: %s', client.get_result())
            rospy.sleep(5.0)
    print "ROS no longer running, cancel goals"
    client.cancel_all_goals()


if __name__ == '__main__':
    #rospy.Subscriber("chatter", String, callback)
    #rospy.loginfo("%s", String)
    #rospy.sleep(3.0)
    rooms_dict = csv2dict(os.path.join(
                            os.environ['ROS_WORKSPACE'].split(':')[0],
                            'simple_navigation_goals',
                            'config', 'rooms.csv'))
    try:
        listener()
    except KeyboardInterrupt:
        print "Caught Interupt, cancelling goals..."
        client.cancel_all_goals()
    #rospy.spin()
