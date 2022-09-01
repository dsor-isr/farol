cmake_minimum_required(VERSION 3.1)
project(auv_sim)

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 roscpp
 dsor_msgs
 nav_msgs
 message_generation
)

find_package(
  # Eigen Library required
  Eigen3 REQUIRED
)

# Custom services from the simulation package
add_service_files(
  FILES
  StartPause.srv
)

# Dependencies for building the custom services
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 dsor_msgs
 nav_msgs
 message_runtime
)

# Compilation flags
add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/ros
 include/algorithms
 ${catkin_INCLUDE_DIRS}
 # Eigen Library is required for this package
 ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node

  # Library specific files
  src/algorithms/AUV.cpp

  # ROS specific details 
  src/ros/SimulationNode.cpp
  src/ros/Subscribers.cpp
  src/ros/Publishers.cpp
  src/ros/Services.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)