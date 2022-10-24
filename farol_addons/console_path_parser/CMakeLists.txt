cmake_minimum_required(VERSION 3.1)
project(console_path_parser)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 farol_msgs
 auv_msgs
 roscpp
 
 # Farol gimmicks - an utils library
 farol_gimmicks_library

 # Package that implements paths
 dsor_paths
 path_following
)

catkin_package(
 CATKIN_DEPENDS
)
find_package(Eigen3 REQUIRED) 

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/ros
 ${EIGEN3_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
)

add_executable(
	${PROJECT_NAME}_node 
	
  # ROS files
  src/ros/ConsolePathParserNode.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
