#!/usr/bin/env python

# ROS packages (python)
import rospy

from geometry_msgs.msg import Twist

class MotionPlanner:
    def __init__(self):
        self.velocity = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def move(self, dir):
        if dir == 0:
            self.velocity.linear.x = 0
            self.velocity.angular.z = 0
            rospy.loginfo('Lin. vel. = 0')

        if dir == 1:
            self.velocity.linear.x = 0.2
            self.velocity.angular.z = 0
            rospy.loginfo('Lin. vel. = 0.2')

        if dir == 2:
            self.velocity.linear.x = .075
            self.velocity.angular.z = .15
            rospy.loginfo('Lin. vel. = 0.1 - Ang. vel. = 0.15')

        if dir == 3:
            self.velocity.linear.x = .075
            self.velocity.angular.z = -.15
            rospy.loginfo('Lin. vel. = 0.1 - Ang. vel. = -0.15')

        self.publisher.publish(self.velocity)