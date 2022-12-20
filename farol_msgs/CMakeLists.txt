cmake_minimum_required(VERSION 3.1)
project(farol_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  std_msgs 
  message_generation 
  geometry_msgs
  auv_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
)


# Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   auv_msgs
)

catkin_package(	CATKIN_DEPENDS message_runtime)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
