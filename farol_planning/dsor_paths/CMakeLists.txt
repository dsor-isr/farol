cmake_minimum_required(VERSION 3.1)
project(dsor_paths)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 auv_msgs
 roscpp
 message_generation

 # Farol gimmicks library is used for parameter loading
 farol_gimmicks_library
)

find_package(
 # Eigen Library required
 Eigen3 REQUIRED
)

# Include here your services for each path section
add_service_files(
  DIRECTORY srv
)

# Include here your custom messages
add_message_files(
  DIRECTORY msg
)

# Put here the message dependencies
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
 include/algorithms/path_sections
 include/algorithms/speed_profiles
 ${catkin_INCLUDE_DIRS}
 # Eigen Library is required for this package
 ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node 
  
  # ROS Files
  src/ros/PathNode.cpp
  src/ros/PathServices.cpp

  # Speed Profiles Library files
  src/algorithms/speed_profiles/Speed.cpp
  src/algorithms/speed_profiles/ConstRabbitSpeed.cpp
  src/algorithms/speed_profiles/ConstVehicleSpeed.cpp
   
  # Path Library files
  src/algorithms/path_sections/Arc2D.cpp
  src/algorithms/path_sections/Circle2D.cpp
  src/algorithms/path_sections/Bernoulli.cpp
  src/algorithms/path_sections/Line.cpp
  src/algorithms/path_sections/PathSection.cpp
  src/algorithms/path_sections/Polynomial5.cpp
  src/algorithms/path_sections/Sinusoid2D.cpp
  src/algorithms/Path.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
