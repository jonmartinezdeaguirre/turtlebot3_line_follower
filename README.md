# Turtlebot3 Line Follower

This ROS Noetic project simulates (in Gazebo environment) or executes a simple line follower module for Turtlebot3. It also enables to record the robot camera with and without the detection bounding rectangle shown below.

<br>

<p align="center">
  <img src="./assets/robot_view.gif" alt="animated"/>
</p>

<br>

<p align="center">
  <img src="https://img.shields.io/github/license/jonmartinezdeaguirre/turtlebot3_line_follower?label=License&style=for-the-badge&color=yellow" href="https://opensource.org/licenses/MIT"/>
  <img src="https://img.shields.io/github/languages/top/jonmartinezdeaguirre/turtlebot3_line_follower?style=for-the-badge&color=green"/>
</p>

## Overview

The main purpose of a line follower module is to enable a robot to autonomously follow a path defined by a line or a boundary (in this simple case, a path defined by red tape). In general, a line follower module consists of a set of sensors, such as camera sensors, that can be used to detect the position of a line or a boundary. Based on the sensor data, the module then computes an appropriate control action, such as adjusting the speed and direction of the robot's movement, to keep the robot on the desired path.

Line follower modules are commonly used in robotics applications such as automated guided vehicles (AGVs), industrial robots, and unmanned ground vehicles (UGVs). They are particularly useful in situations where a robot needs to navigate along a predefined path, such as in manufacturing or warehouse environments.

## Dependencies

`
rospy
std_msgs
message_generation
opencv-python
numpy
`

## World

In order to perform the simulation, a maze-like world was created. This world contains some basic walls and a red path to follow as seen below.

<br>

<p align="center">
  <img width="60%" src="./assets/world.png"/>
</p>
