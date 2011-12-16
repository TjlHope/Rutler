#!/usr/bin/env python

from __future__ import division

import math
from collections import namedtuple
import json

import roslib
roslib.load_manifest('kinect')
import rospy
from std_msgs.msg import Empty, Int8, String
from kinect.msg import User


PI_180 = math.pi / 180

Polar = namedtuple("Polar", "r theta")
PolarID = namedtuple("PolarID", "id pos")


class Vision(object):
    def __init__(self):
        # Start without any users
        self.active_user = None
        self.users = {}
        # Start in the center
        self.position = 0
        # Set up ROS topics
        rospy.loginfo("Connect to servo...")
        rospy.Subscriber("servo_position", Int8, self.servo_watcher)
        self.servo_publisher = rospy.Publisher("servo", Int8)
        rospy.loginfo("Connect to kinect...")
        rospy.Subscriber("kinect_users", User, self.kinect_watcher)
        rospy.loginfo("Connect to interface...")
        rospy.Subscriber("new_user", Empty, self.new_user)
        self.user_publisher = rospy.Publisher("user_position", String)
        self.loop = rospy.timer.Rate(10)

    def servo_watcher(self, position):
        """Callback function to process data from the servo."""
        if position.data:
            self.position = position.data * PI_180

    def kinect_watcher(self, kinect_user):
        """Callback function to process data from the kinect."""
        if kinect_user.id:      # all kinect IDs are >= 1
            self.users[kinect_user.id] = kinect_user

    def new_user(self, signal):
        """Indicate a new user needs to be detected."""
        self.active_user = None
        rospy.loginfo("Told to get new user.")

    def user_position(self, pos):
        """Combine Servo position with user position to give user position
        relative to the robot, rather than the kinect. Returned in Polar form.
        """
        r = math.sqrt(pos.x ** 2 + pos.z ** 2)
        theta = math.atan(pos.x/ pos.z) + self.position
        return Polar(r, theta)

    def get_new_user(self):
        """Iterate over current active users to find one within given
        parameters. If several, return the best fit."""
        rospy.loginfo("Trying to get new user...")
        for user in self.users.itervalues():
            if user.active:     # only care about active users
                # see how close they are
                pos = self.user_position(user.pos)
                if (pos.r < 2000 and abs(pos.theta < 0.4) and   # close
                        (math.sqrt(sum([user.vel.x ** 2, user.vel.y ** 2,
                                        user.vel.z ** 2])) < 400)): # slow
                    if (not self.active_user or        # best candidate
                            (abs(pos.theta) < abs(self.active_user.pos.theta))):
                        self.active_user = PolarID(user.id, pos)
                        rospy.loginfo("Got new user %s", user.id)
                    else:
                        rospy.loginfo("Already have a better avtive user.")
                else:
                    rospy.loginfo("New User %s not in parameters.", user.id)

    def spin(self):
        """Main loop for the vision controller. Named for ROS similarity."""
        rospy.loginfo("Spin...")
        while not rospy.is_shutdown():
            if not self.active_user:
                self.get_new_user()
            if self.active_user:
                user = self.users[self.active_user.id]
                if user.active:
                    new_pos = self.user_position(user.pos)
                    self.user_publisher.publish(
                            json.dumps(dict(zip(new_pos._fields,
                                                new_pos))))
                    if (not math.isnan(new_pos.theta) and
                            abs(new_pos.theta - self.position) > 0.08):
                        servo_msg = Int8(int(new_pos.theta / PI_180))
                        self.servo_publisher.publish(servo_msg)
                    self.active_user = PolarID(self.active_user.id, new_pos)
                    rospy.loginfo("Publish user:\n%s", self.active_user)
                else:
                    self.active_user = None
                    self.servo_publisher.publish(Int8(0))
                    rospy.loginfo("Lost user %s", user.id)
            self.loop.sleep()


if __name__ == '__main__':
    rospy.init_node('vision')
    vision_node = Vision()
    vision_node.spin()
