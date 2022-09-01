cmake_minimum_required(VERSION 3.1)
project(inner_loops_pid)

find_package(catkin REQUIRED COMPONENTS
  farol_msgs
  dsor_utils
  dsor_msgs
  roscpp
  auv_msgs
  farol_gimmicks_library
  message_generation
)

add_compile_options(-std=c++17 -Wall -O3)

add_service_files(
  FILES
  ChangeFFGains.srv
  ChangeInnerGains.srv
  ChangeInnerLimits.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS dsor_utils dsor_msgs roscpp farol_gimmicks_library auv_msgs)

include_directories(include include/ros include/controllers)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} 
  src/ros/innerloops_node.cpp 
  src/ros/innerloops.cpp 
  src/controllers/ros_controller.cpp 
  src/controllers/pid_controller.cpp 
  src/ros/safeties.cpp)
add_dependencies(${PROJECT_NAME} ${inner_loops_forces_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)