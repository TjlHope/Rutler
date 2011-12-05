#!/usr/bin/env python

import roslib; roslib.load_manifest('node_node')
import rospy
from std_msgs.msg import String
import sys
import shlex
import subprocess as sp
import threading as th
import json

class NodeProcess(object):
    def __init__(self, stdin=sp.PIPE, stdout=sp.PIPE, stderr=sp.PIPE, 
                 name='node'):
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

    def get_output(self):
        while not rospy.is_shutdown():
            try:
                line = self.proc.stdout.readline()
                out = json.loads(line)
                self.pub.publish(String(json.dumps(out)))
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
    node.pub = rospy.Publisher('user_input', String)
    rospy.spin()

if __name__ == '__main__':
    node = NodeProcess(name=
        'node /home/jkimbo/www/rutler-interface/app.js')
    watcher(node)

