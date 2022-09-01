cmake_minimum_required(VERSION 3.1)
project(evo_tools)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs
 farol_msgs
 auv_msgs
 roscpp
 farol_gimmicks_library
 geometry_msgs
 message_filters
 tf2
 tf2_ros
 dmac
)

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIR})

catkin_package(
 CATKIN_DEPENDS
 dmac
 DEPENDS GeographicLib
)

catkin_install_python(PROGRAMS src/pos2simmodem.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_compile_options(-std=c++11)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Eigen3_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# dmac to farol
add_executable(dmac_to_farol_node src/DmacToFarolNode.cpp)
add_dependencies(dmac_to_farol_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dmac_to_farol_node ${catkin_LIBRARIES})
