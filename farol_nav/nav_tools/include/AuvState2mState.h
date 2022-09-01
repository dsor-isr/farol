/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico 
*/
#ifndef CATKIN_WS_AUVSTATE2MSTATE_H
#define CATKIN_WS_AUVSTATE2MSTATE_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>
#include "LowPassFilter.h"

#include <ros/ros.h>

// ROS messages and stuff
#include <auv_msgs/NavigationStatus.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/Pressure.h>
#include <sensor_msgs/NavSatFix.h>

// Farol Libraries and msgs
#include <farol_gimmicks_library/FarolGimmicks.h>

// TFs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class AuvState2mState
{
public:
  /* -------------------------------------------------------------------------*/
  /**
   * @brief   Construtor  
   *
   * @Param nodehandle
   * @Param nodehandle_private
   */
  /* -------------------------------------------------------------------------*/
	AuvState2mState(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
	~AuvState2mState();

private:
	ros::NodeHandle nh_, nh_private_;

	// @.@ Subsctibers
    ros::Subscriber sub_inside_pressure_, sub_auv_state_, sub_gps_status_;

	// @.@ Publishers
	ros::Publisher mstate_pub_;

	// @.@ Timers
	ros::Timer timer_in_pressure_;

	// +.+ Problem variables
	farol_msgs::Pressure in_pressure;
	LowPassFilter* insidePressure;
	double in_press, in_press_dot;
	unsigned int gps_status_{1};

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
	void initializePublishers();
  
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
	void loadParams();

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
	void mStateBroadcasterCallback(const auv_msgs::NavigationStatus &msg);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
	void insidePressureCallback(const farol_msgs::Pressure &msg);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
	void mGPSStatusCallback(const sensor_msgs::NavSatFix &msg);
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void insidePressureTimer(const ros::TimerEvent& event);

};
#endif //AuvState2mState_H
