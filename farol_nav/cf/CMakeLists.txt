cmake_minimum_required(VERSION 3.1.0)
project(cf)

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 roscpp
 dsor_msgs
 auv_msgs
 farol_msgs
 farol_gimmicks_library
 cmake_modules
 message_generation
)

find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 farol_gimmicks_library
 dsor_msgs
 farol_msgs
)

catkin_package(	CATKIN_DEPENDS message_runtime)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/cf_ros
 include/cf_algorithms
 include/ros
 ${EIGEN3_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
 ${farol_gimmicks_library_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} src/cf_ros/CfNode.cpp src/cf_algorithms/CfAlgorithm.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
