/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico 
*/
#ifndef CATKIN_WS_GNSS2UTMOUTLIER_H
#define CATKIN_WS_GNSS2UTMOUTLIER_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h>

// ROS messages and stuff
#include <sensor_msgs/NavSatFix.h>
#include <dsor_msgs/Measurement.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

// Farol Libraries and msgs
#include <farol_gimmicks_library/FarolGimmicks.h>

// TFs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Third Party Libraries
#include <GeographicLib/UTMUPS.hpp>

class Gnss2UtmOutlier
{
public:
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Constructor
   *
   * @Param nodehandle
   * @Param nodehandle_private
   */
  /* -------------------------------------------------------------------------*/
	Gnss2UtmOutlier(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
	~Gnss2UtmOutlier();

private:
	ros::NodeHandle nh_, nh_private_;

	// @.@ Subsctibers
    ros::Subscriber gnss_sub_;

	// @.@ Publishers
	ros::Publisher state_pub_;
  dsor_msgs::Measurement utm_;

	// +.+ package.xml Parameters from Yaml
  double p_default_depth_;
  ros::Timer timer_gps_outlier_;
  ros::Timer timer_usbl_back_;
  ros::ServiceServer enable_gps_outlier_srv_;
  ros::ServiceServer enable_usbl_delay_srv_;

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
	void initializeSubscribers();
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
  void initializeTimers();
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
  void initializePublishers();
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
  void initializeServices();
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
  void loadParams();

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
	void gnssBroadcasterCallback(const sensor_msgs::NavSatFix &msg);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerGPSOutlierCallback(const ros::TimerEvent &event);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerUSBLDelayCallback(const ros::TimerEvent &event);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param req
   * @Param res
   *
   * @Returns   
   */
  /* -------------------------------------------------------------------------*/
  bool enableGPSService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param req
   * @Param res
   *
   * @Returns   
   */
  /* -------------------------------------------------------------------------*/
  bool enableUSBLService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
 };
#endif 
