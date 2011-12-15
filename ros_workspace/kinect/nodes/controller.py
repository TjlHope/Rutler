#!/usr/bin/env python

import math
from collections import namedtuple

import roslib; roslib.load_manifest('kinect')
import rospy
from std_msgs.msg import Empty, Int8
from kinect.msg import User, Polar


PolarID = namedtuple("PolarID", "id pos")


class Vision(object):
    def __init__(self):
        # Start without any users
        self.active_user = None
        self.users = {}
        # Set up ROS topics
        rospy.Subscriber("servo_position", Int8, self.servo_watcher)
        rospy.Subscriber("kinect_users", User, self.kinect_watcher)
        rospy.Subscriber("new_user", Empty, self.new_user)
        self.user_publisher = rospy.Publisher("user_position", Polar)
        self.loop = rospy.timer.Rate(10)

    def servo_watcher(self, position):
        """Callback function to process data from the servo."""
        if position.data:
            self.position = position.data

    def kinect_watcher(self, kinect_user):
        """Callback function to process data from the kinect."""
        if kinect_user.id:      # all kinect IDs are >= 1
            if kinect_user.active:      # don't bother if they're not active
                self.users[kinect_user.id] = kinect_user

    def new_user(self, signal):
        """Indicate a new user needs to be detected."""
        self.active_user = None

    def user_position(self, user):
        """Combine Servo position with user position to give user position
        relative to the robot, rather than the kinect. Returned in Polar form.
        """
        pos = Polar(0, 0)
        return pos

    def get_new_user(self):
        """Iterate over current active users to find one within given
        parameters. If several, return the best fit."""
        for user in self.users.itervalues():
            if user.active:     # only care about active users
                # see how close they are
                pos = self.user_position(user.pos)
                if (pos.r < 2000 and abs(pos.theta < 0.5) and   # close
                        math.sqrt(sum(user.vel.x ** 2, user.vel.y ** 2,
                                      user.vel.z ** 2)) and     # slow
                        (not self.active_user or        # best candidate
                         (abs(pos.theta) < abs(self.active_user.pos.theta)))):
                    self.active_user = PolarID(user.id, pos)

    def spin(self):
        """Main loop for the vision controller. Named for ROS similarity."""
        while rospy.ok():
            if not self.active_user:
                self.get_new_user()
            if self.active_user:
                #TODO: calculate and send position
                user = self.users[self.active_user.id]
                self.active_user = PolarID(self.active_user.id,
                                           self.user_position(user.pos))
                self.user_publisher(self.active_user.pos)
            self.loop.sleep()


if __name__ == '__main__':
    rospy.init_node('vision')
    vision_node = Vision()
    vision_node.spin()

