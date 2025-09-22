cmake_minimum_required(VERSION 3.1.0)
project(farol_docking)

find_package(catkin REQUIRED
  COMPONENTS
  std_msgs 
  auv_msgs
  dsor_msgs
  farol_msgs
  sensor_msgs
  drivers_msgs
  message_generation
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  farol_gimmicks_library
  cmake_modules
  waypoint
  inner_loops_pid
  topic_tools
)
 
## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
)
  
  
  # Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
)
  
generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)
   

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES farol_docking
  CATKIN_DEPENDS
  tf2_geometry_msgs   
  tf2_ros
  roscpp
  farol_gimmicks_library
  dsor_msgs
  message_runtime
  topic_tools
)
    
add_compile_options(-std=c++17 -Wall -O3) 
    
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

    
add_executable(docking_filter src/filter/docking_filter_node.cpp src/filter/docking_filter.cpp)
add_dependencies(docking_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_filter ${catkin_LIBRARIES})

add_executable(outer_loop src/outer_loop/outer_loop_node.cpp src/outer_loop/trajectory.cpp )
add_dependencies(outer_loop ${catkin_EXPORTED_TARGETS})
add_dependencies(outer_loop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(outer_loop ${catkin_LIBRARIES})

add_executable(trajectory_tracking src/trajectory_tracking/trajectory_tracking_node.cpp src/trajectory_tracking/se3_tracker.cpp)
add_dependencies(trajectory_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_tracking ${catkin_LIBRARIES} )


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
