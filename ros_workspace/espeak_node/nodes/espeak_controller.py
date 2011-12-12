#!/usr/bin/env python

import shlex
import subprocess as sp
import threading as th

import roslib; roslib.load_manifest('espeak_node')
import rospy
from std_msgs.msg import String


class EspeakProcess(object):
    def __init__(self, stdin=sp.PIPE, stdout=sp.PIPE, stderr=sp.STDOUT, 
                 name='espeak'):
        self.stdin = stdin
        self.stdout = stdout
        self.stderr = stderr
        if isinstance(name, basestring):
            name = shlex.split(name)
        self.proc = sp.Popen(name, stdin=stdin, stdout=stdout, stderr=stderr)
        self.out = th.Thread(target=self.get_output)
        self.out.start()

    def sendCommand(self, data):
        if data.data:
            self.proc.stdin.write(data.data)
            rospy.loginfo(rospy.get_name()+
                        " I heard: %s",data.data)

    def get_output(self):
        while not rospy.is_shutdown():
            out = self.proc.stdout.readline()
            rospy.logerr("I got error from espeak: %s", out)


def watcher(node):
    rospy.init_node('espeak_node', anonymous=True)
    rospy.Subscriber('talk', String, node.sendCommand)
    rospy.spin()

if __name__ == '__main__':
    node = EspeakProcess(name='espeak -s 130 -v en-sc')
    watcher(node)
