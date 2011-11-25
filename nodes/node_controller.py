#!/usr/bin/env python

import roslib; roslib.load_manifest('rutler_interface')
import rospy
from std_msgs.msg import String
import sys
import shlex
import subprocess as sp

class NodeProcess(object):
    def __init__(self, stdin=sp.PIPE, stdout=sp.PIPE, stderr=sp.PIPE, 
                 name='node'):
        self.stdin = stdin
        self.stdout = stdout
        self.stderr = stderr
        if isinstance(name, basestring):
            name = shlex.split(name)
        self.proc = sp.Popen(name, stdin=stdin, stdout=stdout, stderr=stderr)
    def sendCommand(self, data):
        self.proc.stdin.write(data.data)
        rospy.loginfo(rospy.get_name()+
                    " I heard %s",data.data)

def watcher():
    rospy.init_node('watcher', anonymous=True)
    rospy.Subscriber("chatter", String, node.sendCommand)
    rospy.spin()

if __name__ == '__main__':
    node = NodeProcess(name='node /home/jkimbo/www/rutler-interface/app.js')
    watcher()
    #print node.proc.stdout.readline()
    #node.proc.terminate()

