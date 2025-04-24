/** 
 *  @file   docking_utils.h 
 *  @brief  utils for docking
 *  @author Ravi Regalo
 *  @date   today :)
 ***********************************************/
#ifndef CATKIN_WS_DOCKINGUTILS
#define CATKIN_WS_DOCKINGUTILS


// @.@ ROS Libraries
#include <ros/ros.h>
// @.@ TF's
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
 #include <farol_gimmicks_library/FarolGimmicks.h>


// @.@ ROS Messages
#include <auv_msgs/NavigationStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>  
#include <dsor_msgs/Measurement.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/mUSBLFix.h>


// @.@ Third Party Libraries
#include <Eigen/Eigen>
#include <GeographicLib/GeoCoords.hpp>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <cmath> 
#include <deque>
#include <vector>
#include <bitset>



void printDeque(const std::deque<double>& dq);

double radiansToDegrees360(double radians);

Eigen::Matrix2d Rot2D(double yaw);

double wrapToPi(double angle);

double wrapTo2Pi(double angle);

double sigma_e(double input);


#endif //CATKIN_WS_DOCKINGUTILS

