cmake_minimum_required(VERSION 3.1)
project(comms_state_broadcaster)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 auv_msgs
 farol_msgs
 roscpp
 farol_gimmicks_library
 geometry_msgs
# geo_conversion 
 message_filters
 tf2
 tf2_ros
 roscpp
)

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIR})

catkin_package(
 CATKIN_DEPENDS
 DEPENDS GeographicLib
)

add_compile_options(-std=c++11) 

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#find_package(GeographicLib REQUIRED)    
#include_directories(${GeographicLib_INCLUDE_DIR})

# gnss_to_state
add_executable(gnss_to_state src/Gnss2State.cpp)
add_dependencies(gnss_to_state ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_to_state ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

# state_to_gnss
#add_executable(state_to_gnss src/State2Gnss.cpp)
#add_dependencies(state_to_gnss ${catkin_EXPORTED_TARGETS})
#target_link_libraries(state_to_gnss ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

# usblfix_to_state
#add_executable(usblfix_to_state src/UsblFix2State.cpp)
#add_dependencies(usblfix_to_state ${catkin_EXPORTED_TARGETS})
#target_link_libraries(usblfix_to_state ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
