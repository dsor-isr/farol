cmake_minimum_required(VERSION 3.1)
project(farol_bringup)

find_package(catkin REQUIRED COMPONENTS
  farol_msgs
  rospy
  roscpp
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)