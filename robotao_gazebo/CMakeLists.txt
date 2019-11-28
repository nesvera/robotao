# Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 2.8.3)
project(robotao_gazebo)

# Find catkin packages and libraries for catkin and system dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Setup for python modules and scripts

# Declare ROS messages, services and actions

# Declare ROS dynamic reconfigure parameters

# Declare catkin specific configuration to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

# Build
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install
install(DIRECTORY config launch world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test
