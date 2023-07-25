cmake_minimum_required(VERSION 3.1)
project(interrogation_scheme)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 farol_msgs
 roscpp
 dmac
 farol_gimmicks_library
 message_generation
)

add_message_files(
   DIRECTORY msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
 CATKIN_DEPENDS
    roscpp
    std_msgs
    farol_msgs
    dmac
    message_runtime
)


add_compile_options(-std=c++11) 

include_directories(
 include/pinger_ros
 include/replier_ros
 include/silent_ros
 ${catkin_INCLUDE_DIRS}
)

add_executable(pinger_sync_node src/pinger_ros/PingerNode.cpp)
add_dependencies(pinger_sync_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pinger_sync_node ${catkin_LIBRARIES})

add_executable(replier_sync_node src/replier_ros/ReplierNode.cpp)
add_dependencies(replier_sync_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(replier_sync_node ${catkin_LIBRARIES})

add_executable(silent_sync_node src/silent_ros/SilentNode.cpp)
add_dependencies(silent_sync_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(silent_sync_node ${catkin_LIBRARIES})