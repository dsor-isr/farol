cmake_minimum_required(VERSION 3.1)
project(sensor_fusion)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 auv_msgs
 dsor_msgs
 farol_msgs
 roscpp
 tf2
 tf2_geometry_msgs
 tf2_ros
 farol_gimmicks_library
 cmake_modules
)

add_service_files(
  FILES
  SetVCurrentVelocity.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 tf2_geometry_msgs
 tf2_ros
# GeographicLib
)

find_package(GeographicLib REQUIRED)    
include_directories(${GeographicLib_INCLUDE_DIR})

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/ros
 include/algorithms
 ${catkin_INCLUDE_DIRS}
 ${Eigen3_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} src/ros/FiltersNode.cpp src/algorithms/HorizontalFilter.cpp src/algorithms/VerticalFilter.cpp src/algorithms/RotationalFilter.cpp src/algorithms/DeadReckoning.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})