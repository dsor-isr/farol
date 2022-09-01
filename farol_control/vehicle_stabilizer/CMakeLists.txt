cmake_minimum_required(VERSION 3.1)
project(vehicle_stabilizer)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 roscpp
 std_msgs

 # Farol gimmicks library 
 farol_gimmicks_library
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 std_msgs
 farol_gimmicks_library
)

# Generate very optimized code? Yes, please!
add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  # ROS node
  src/VehicleStabilizerNode.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})