cmake_minimum_required(VERSION 3.1.0)
project(farol_docking)

find_package(catkin REQUIRED
    COMPONENTS
    std_msgs 
    auv_msgs
    dsor_msgs
    farol_msgs
    gazebo_msgs
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
  # DIRECTORY msg
	FILES
  ControllerDebug.msg
  FilterDebug.msg
  dState.msg
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
  INCLUDE_DIRS filter/include controller/include
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
  filter/include 
  controller/include
  utils/include
  ${catkin_INCLUDE_DIRS}
)
     
    
add_executable(docking_filter filter/src/DockingFilterNode.cpp filter/src/DockingHFilter.cpp filter/src/DockingRFilter.cpp filter/src/DockingVFilter.cpp utils/src/docking_utils.cpp )
# add_dependencies(docking_filter ${catkin_EXPORTED_TARGETS})
add_dependencies(docking_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_filter ${catkin_LIBRARIES})

add_executable(docking_controller controller/src/DockingControllerNode.cpp controller/src/DockingController.cpp utils/src/docking_utils.cpp)
# add_dependencies(docking_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(docking_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_controller ${catkin_LIBRARIES})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
