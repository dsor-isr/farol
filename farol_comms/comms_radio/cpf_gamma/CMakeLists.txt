cmake_minimum_required(VERSION 3.1)
project(cpf_gamma)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs
 farol_msgs
 rospy
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
)
