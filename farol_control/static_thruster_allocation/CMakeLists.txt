cmake_minimum_required(VERSION 3.1)
project(static_thruster_allocation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dsor_msgs
  auv_msgs
  farol_gimmicks_library
)

add_compile_options(-std=c++17 -Wall -O3)

catkin_package(CATKIN_DEPENDS roscpp farol_gimmicks_library auv_msgs dsor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(static_thruster_allocation src/thruster_allocation.cpp)
add_dependencies(static_thruster_allocation ${static_thruster_allocation_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(static_thruster_allocation ${catkin_LIBRARIES})