cmake_minimum_required(VERSION 3.1.0)
project(bottom_following)

find_package(catkin 
  # ROS components
  REQUIRED COMPONENTS
  rospy
  std_msgs
  farol_msgs
  dsor_msgs
  geometry_msgs
  auv_msgs
  message_generation 
)

add_message_files(
	DIRECTORY msg
)

# add_service_files(
# 	DIRECTORY srv
# )

catkin_python_setup()

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   auv_msgs
)

catkin_package(	CATKIN_DEPENDS message_runtime)