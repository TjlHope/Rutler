#!/usr/bin/env python

import sys
import os
import shlex
import subprocess as sp
import threading as th
import json

import roslib; roslib.load_manifest('node_node')
import rospy
from std_msgs.msg import String


class NodeProcess(object):
    def __init__(self, stdin=sp.PIPE, stdout=sp.PIPE, stderr=sp.PIPE, 
                 name='node'):
        with open(os.path.join(os.environ['ROS_WORKSPACE'].split(':')[0],
                               'speech_recognition', 'config',
                               'map_people_room.json')) as fl:
            self.map_people_room = json.load(fl)
        self.stdin = stdin
        self.stdout = stdout
        self.stderr = stderr
        if isinstance(name, basestring):
            name = shlex.split(name)
        self.proc = sp.Popen(name, stdin=stdin, stdout=stdout, stderr=stderr)
        self.out = th.Thread(target=self.get_output)
        self.err = th.Thread(target=self.get_error)
        self.out.start()
        self.err.start()

    def sendCommand(self, data):
        if data.data:
            self.proc.stdin.write(data.data)
            rospy.loginfo(rospy.get_name()+
                        " I heard %s",data.data)

    def sendRecognition(self, data):
        if data.data:
            if not data.data[1:-1].isdigit():
                try:
                    data.data = self.map_people_room[data.data]
                except KeyError:
                    # Invalid person
                    return
            output = '{"recog": "'+data.data+'"}'
            self.proc.stdin.write(output)
            rospy.loginfo(rospy.get_name()+
                        " I heard %s",data.data)

    def sendCommands(self, data):
        if data.data:
            output = '{"speechCommand": "'+data.data+'"}'
            self.proc.stdin.write(output)
            rospy.loginfo(rospy.get_name()+
                        " I heard %s",data.data)

    def sendPosition(self, data):
        if data.data:
            self.proc.stdin.write(data.data);
            rospy.loginfo(rospy.get_name()+
                        " I heard %s",data.data)

    def get_output(self):
        while not rospy.is_shutdown():
            try:
                line = self.proc.stdout.readline()
                out = json.loads(line)
                self.pub.publish(String(json.dumps(out)))
                self.move.publish(String(json.dumps(out)))
                self.talk.publish(String(json.dumps(out)))
                rospy.loginfo("JSON: %s", out)
            except ValueError:
                out = line
            rospy.loginfo("I got from Node: %s", out)

    def get_error(self):
        while not rospy.is_shutdown():
            err = self.proc.stderr.readline()
            rospy.logerr("I got error from Node: %s", err)


def watcher(node):
    rospy.init_node('watcher', anonymous=True)
    rospy.Subscriber("interface", String, node.sendCommand)
    rospy.Subscriber("rutler_status", String, node.sendCommand)
    rospy.Subscriber("speech_names/output", String, node.sendRecognition)
    rospy.Subscriber("speech_commands/output", String, node.sendCommands)
    rospy.Subscriber("user_position", String, node.sendPosition)
    node.pub = rospy.Publisher('user_input', String)
    node.move = rospy.Publisher('move_rutler', String)
    node.talk = rospy.Publisher('speak_input', String)
    rospy.spin()


if __name__ == '__main__':
    node = NodeProcess(name=
        'node /home/rutler/rutler-interface/app.js')
    watcher(node)

