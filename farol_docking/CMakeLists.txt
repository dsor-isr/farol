cmake_minimum_required(VERSION 3.1.0)
project(farol_docking)

find_package(catkin REQUIRED
    COMPONENTS
    std_msgs 
    auv_msgs
    dsor_msgs
    farol_msgs
    message_generation
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    farol_gimmicks_library
    cmake_modules
)
 
## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
)
  
  
  # Generate services in the 'srv' folder
  # add_service_files(
    # 	DIRECTORY srv
    # )
    
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
  # GeographicLib
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

#add_executable(docking_controller src/controller/DockingControllerNode.cpp src/controller/DockingController.cpp)
# add_dependencies(docking_controller ${catkin_EXPORTED_TARGETS})
#add_dependencies(docking_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(docking_controller ${catkin_LIBRARIES})

add_executable(inner_loops src/inner_loops/inner_loops_node.cpp src/inner_loops/smc.cpp src/inner_loops/pid.cpp)
add_dependencies(inner_loops ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inner_loops ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
