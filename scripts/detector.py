#!/usr/bin/env python

# Python packages
import cv2
import numpy as np

# ROS packages (python)
import rospy
from cv_bridge import CvBridge

# Messages
from sensor_msgs.msg import Image

class LineDetector:
    def __init__(self):
        self.video_data = []
        self.video_writer = cv2.VideoWriter('../recordings/detection.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (320, 240))
        self.bridge = CvBridge()

        self.detection = Image()
        self.publisher = rospy.Publisher('line_follower', Image, queue_size=1)

    def read_image(self, message: Image):
        '''
        Reads ROS message and turns it into numpy array.
        It also applies a Gaussian blur and turns the blurred image into HSV color format.

        Inputs
        ---
            message: sensor_msgs.msg.Image
                Sensor image message
        '''

        self.image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.blurred = cv2.GaussianBlur(self.image, ksize=(3, 3), sigmaX=.1, sigmaY=.1)
        self.blurred_hsv = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)
        
        self.height, self.width, _ = self.image.shape

    def get_direction(self, message=None, line_color='red', tol=10):
        '''
        Given an image message, it returns the direction the robot must take in order to follow the line.
        It calculates the direction mainly based on the color of the line.

        Inputs
        ---
            message: sensor_msgs.msg.Image
                Sensor image message

            line_color: str
                Line color: "red" or "black"
            
            tol: int, default = 10
                Admitted tolerance to calculate direction

        Outputs
        ---
            dir: int
                Direction the robot must take:
                    Stop = 0; Straight = 1; Left = 2; Right = 3;
        '''
        
        if message:
            self.read_image(message)

        if line_color == 'red':
            lower = np.array([0, 100, 100])
            upper = np.array([10, 255, 255])

        if line_color == 'black':
            lower = np.array([0, 0, 0])
            upper = np.array([179, 20, 155])
        
        mask = cv2.inRange(self.blurred_hsv, lower, upper)

        search_y = int(self.height*2/5)
        mask[:search_y, ] = 0
        moments = cv2.moments(mask)

        try:
            cx = int(moments['m10']/moments['m00'])

            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(max(contours, key=np.size))

            cv2.rectangle(self.image, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), (0, 255, 0), 2)
            cv2.putText(self.image, 'Detected line', (rect_x - 2, rect_y - 8), cv2.FONT_HERSHEY_DUPLEX, .4, (0, 255, 0))

            self.detection = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(self.detection)

            self.video_data.append(self.image)

        except ZeroDivisionError:
            cv2.putText(self.image, '[WARNING] No line found', (int(self.width/5), 20), cv2.FONT_HERSHEY_DUPLEX, .5, (0, 0, 255))
            
            self.detection = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(self.detection)

            self.video_data.append(self.image)

            rospy.logwarn('No line found')
            return 0
        
        if cx > self.width/2 - tol and cx < self.width/2 + tol:
            return 1
        if cx < self.width/2 - tol:
            return 2
        if cx > self.width/2 + tol:
            return 3

if __name__ == '__main__':
    image_processor = LineDetector()
    image = cv2.imread('/home/turtlepc/Documentos/project/img/0.png')
    image_processor.test(image, 'black')