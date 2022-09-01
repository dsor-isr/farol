cmake_minimum_required(VERSION 3.1)
project(waypoint)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 auv_msgs
 farol_msgs
 roscpp
 farol_gimmicks_library
 message_generation
)

add_compile_options(-std=c++17)

add_service_files(
  FILES
  sendWpType1.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 farol_gimmicks_library
)

include_directories(
 include/waypoint_ros
 include/waypoint_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/waypoint_ros/waypoint.cpp src/waypoint_ros/waypoint_node.cpp src/waypoint_algorithms/wp_heading.cpp src/waypoint_algorithms/wp_loose.cpp src/waypoint_algorithms/wp_standard.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
