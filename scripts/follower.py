#!/usr/bin/env python

# ROS package (python)
import rospy

# Messages
from sensor_msgs.msg import Image

# Module imports
from motion import MotionPlanner
from detector import LineDetector

class Follower:
    def __init__(self):
        self.detector = LineDetector()
        self.motion_planner = MotionPlanner()

        rospy.init_node('line_follower')
        self.rate = rospy.Rate(30)
        self.subscriber = rospy.Subscriber('camera/image', Image, self.camera_callback)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.rate.sleep()

            except rospy.ROSInterruptException:
                self.write_video()
        
    def write_video(self):
        for i in range(len(self.detector.video_data)):
            self.detector.video_writer.write(self.detector.video_data[i])

    def camera_callback(self, msg):
        direction = self.detector.get_direction(message=msg, line_color='red', tol=15)
        self.motion_planner.move(direction)

if __name__ == '__main__':
    follower = Follower()
    follower.run()