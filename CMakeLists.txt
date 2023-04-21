cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_line_follower)

## Find catkin libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  geometry_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Catkin specific configuration ##
catkin_package()

## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)