#!/usr/bin/env python

# ROS package (python)
import rospy

# Messages
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock

# Module imports
from motion import MotionPlanner
from detector import LineDetector

class Follower:
    def __init__(self):
        self.detector = LineDetector()
        self.motion_planner = MotionPlanner()

        rospy.init_node('line_follower')
        self.subscriber = rospy.Subscriber('camera/image', Image, self.camera_callback)
        rospy.spin()

    def camera_callback(self, msg):
        direction = self.detector.get_direction(message=msg, line_color='red', tol=15)
        self.motion_planner.move(direction)

if __name__ == '__main__':
    follower = Follower()