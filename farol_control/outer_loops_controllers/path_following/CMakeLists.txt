cmake_minimum_required(VERSION 3.1)
project(path_following)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 auv_msgs
 farol_msgs
 roscpp
 message_generation
 waypoint

 # Farol gimmicks library 
 farol_gimmicks_library

 # Paths library
 dsor_paths
)

find_package(
  # Eigen Library required
  Eigen3 REQUIRED  
)

# Include here your services for the Path Following
add_service_files(
  FILES
  ResetVT.srv
  SetPF.srv
  StartPF.srv
  StopPF.srv
  UpdateGainsPF.srv
  )


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 # Include for message generation
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
  
  # ROS specific details 
  src/ros/PathFollowingNode.cpp
  src/ros/PathFollowingServices.cpp
	
  # Path Following Algorithms Implementations
  src/algorithms/PathFollowing.cpp
  src/algorithms/Aguiar.cpp
  src/algorithms/Romulo.cpp
  src/algorithms/Brevik.cpp
  src/algorithms/Lapierre.cpp
  src/algorithms/Pramod.cpp
  src/algorithms/Samson.cpp
  src/algorithms/Fossen.cpp
  src/algorithms/Marcelo.cpp
  src/algorithms/RelativeHeading.cpp
)


add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
