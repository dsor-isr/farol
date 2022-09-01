cmake_minimum_required(VERSION 3.1)
project(interrogation_scheme)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 farol_msgs
 roscpp
 dmac
 farol_gimmicks_library
)

catkin_package(
 CATKIN_DEPENDS
    roscpp
    std_msgs
    farol_msgs
    dmac
)

add_compile_options(-std=c++11) 

include_directories(
 include/pinger_ros
 include/replier_ros
 ${catkin_INCLUDE_DIRS}
)

add_executable(pinger_sync_node src/pinger_ros/PingerNode.cpp)
add_dependencies(pinger_sync_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pinger_sync_node ${catkin_LIBRARIES})

add_executable(replier_sync_node src/replier_ros/ReplierNode.cpp)
add_dependencies(replier_sync_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(replier_sync_node ${catkin_LIBRARIES})
