cmake_minimum_required(VERSION 3.1.0)
project(ckf)

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 roscpp
 dsor_msgs
 auv_msgs
 farol_gimmicks_library
 cmake_modules
)

find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 farol_gimmicks_library
 dsor_msgs
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/ckf_ros
 include/ckf_algorithms
 include/ros
 ${EIGEN3_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
 ${farol_gimmicks_library_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} src/ckf_ros/CkfNode.cpp src/ckf_algorithms/CkfAlgorithm.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
