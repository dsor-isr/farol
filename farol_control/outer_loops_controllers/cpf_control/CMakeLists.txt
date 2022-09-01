cmake_minimum_required(VERSION 3.1)
project(cpf_control)

find_package(catkin 
 REQUIRED COMPONENTS
 # ROS components
 std_msgs 
 farol_msgs
 roscpp
 message_generation
 # Farol Gimmicks Libraries
 farol_gimmicks_library
 # Dsor Paths for the message
 dsor_paths
)

find_package(
  # Eigen Library required
  Eigen3 REQUIRED
)

# Include here your services for the Path Following
add_service_files(
  FILES
  StartStop.srv
  ChangeTopology.srv
)

# Put here the message dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 farol_msgs
 #Include for message generation
 message_runtime
)

add_compile_options(-std=c++17 -Wall) 

include_directories(
 include/ros
 include/algorithms
 ${catkin_INCLUDE_DIRS}
 # Eigen Library is required for this package
 ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node 
  # ROS Nodes
  src/ros/CpfNode.cpp
  src/ros/CpfServices.cpp

  # Algorithms
  src/algorithms/CPFControl.cpp
  src/algorithms/EventTriggered.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
