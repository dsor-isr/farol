cmake_minimum_required(VERSION 3.1)
project(sensor_sim)

find_package(catkin REQUIRED
 COMPONENTS
 auv_msgs
 geometry_msgs 
 sensor_msgs 
 std_msgs 
 dsor_msgs
 farol_msgs
 roscpp
 tf2
 tf2_ros
 farol_gimmicks_library
)

find_package(GeographicLib REQUIRED)    
include_directories(${GeographicLib_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
 CATKIN_DEPENDS 
 DEPENDS GeographicLib
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_sim src/SensorSim.cpp)
add_dependencies(sensor_sim ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_sim ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
