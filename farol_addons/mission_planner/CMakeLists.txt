cmake_minimum_required(VERSION 3.1.0)
project(mission_planner)

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 roscpp
 auv_msgs

 message_generation

 # farol_gimmicks_library
 farol_gimmicks_library
)

# Generate messages in the 'msg' folder
add_message_files(
	# DIRECTORY msg
  FILES
  mInterestZone.msg
  mMissionStartedAck.msg
  mNewIZMission.msg
  mGsnifferData.msg
)

add_service_files(
  FILES
  Configs.srv
  InterestZone.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/mission_planner_ros
 include/mission_planner_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mission_planner_ros/MissionPlannerNode.cpp src/mission_planner_algorithms/MissionPlannerAlgorithm.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
