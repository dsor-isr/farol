/**
 * @file   docking_filter_node.hpp
 * @brief  Docking Filter Node class
 * @author Ravi Regalo <ravi.regalo@tecnico.ulisboa.pt>
 * @date   2025-04-25
 * 
 * Description :)
 */
#pragma once

// usefull libraries
#include <vector>
#include <bitset>
#include <algorithm>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"

// ros libraries
#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>  
#include <geometry_msgs/Point.h>  
#include <dsor_msgs/Measurement.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/mUSBLFix.h>
#include <auv_msgs/NavigationStatus.h>
//#include <farol_docking/FilterDebug.h>

// farol libraries
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <farol_docking/utils/docking_utils.hpp>  
#include <farol_docking/filter/docking_filter.hpp>  

 

/**
 * @brief   Interface between ROS and docking filter algorithm
 */
class DockingFilterNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
  * @brief Constructor
  *
  * @param nodehandle
  * @param nodehandle_private
  */
 	DockingFilterNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /**
   * @brief  Destructor
   */
 	~DockingFilterNode();
  
 private:
  /**
   * @brief Load parameters from parameter server 
   */
  void loadParams();

  /**
   * @brief Initialize ROS node Subscribers
   */
  void initializeSubscribers();

  /**
   * @brief  Initialize ROS node Publishers
   */
 	void initializePublishers();
 
  /**
   * @brief  Initialize ROS node Services
   */
 	void initializeServices();

  /**
   * @brief Initialize ROS node Timer  
   */
 	void initializeTimer();

  /**
   * @brief Callback to handle measurement ROS messages. 
   *        They could be AHRS messages or DVL messages
   * @param msg ROS DSOR Measurement msg
   * 
   */
  void measurement_callback(const dsor_msgs::Measurement &msg);

  /**
   * @brief Callback to handle USBL measurement ROS messages. it waits for measurements 
   *        from the vehicle's own USBL and also the docking station USBL fix that comes 
   *        through the acoustims comms
   * @param msg ROS reset USBL fix message
   * 
   */
  void usbl_callback(const farol_msgs::mUSBLFix &msg);

  /**
   * @brief Callback that resets the filter to uninitialized state@
   * @param msg ROS reset msg
   */
  void reset_callback(const std_msgs::Empty &msg);

  /**
   * @brief Function that runs at the node frequency, propagrating the state until 
   *        current time and sending the current state over to ROS
   *
   * @param event 
   */
  void timerIterCallback(const ros::TimerEvent& event);

  
  // ROS node handlers
 	ros::NodeHandle nh_;          
 	ros::NodeHandle nh_private_; 

 	// Subscribers
  ros::Subscriber sub_reset_;
  ros::Subscriber sub_velocity_;
  ros::Subscriber sub_orientation_;
  ros::Subscriber sub_position_;
  ros::Subscriber sub_usbl_fix_;
  ros::Subscriber sub_usbl_accoms_;
  ros::Subscriber sub_dock_inertial_pos_;
  
 	// Publishers
  ros::Publisher state_pub_;
  // ros::Publisher console_state_pub_;
  // ros::Publisher debug_pub_;

  // ROS messages
  auv_msgs::NavigationStatus state_msg_;
  farol_msgs::mState console_state_msg_;
  geometry_msgs::Point usbl_debug_msg;
  //farol_docking::FilterDebug debug_msg_;
  
 	// ROS node iteration timer
 	ros::Timer timer_;    

  // ROS parameters
  double node_frequency_; 
  bool debug_;
  
  // Measurements stuff
  // USBL stuff
  std::bitset<4> usbl_state_;   // keeps track of all the messages that need to be received for an usbl set to be completed
  Sophus::Vector6d usbl_set_;   // holds the set of usbl measurement [auv(range, bearing, elevation), dock(range, bearing, elevation)]
  ros::Time usbl_time_;   // timestamp from the last received usbl message, or a least the time of the latest measured thing
  Eigen::Vector3d dvl_velocity_, ahrs_velocity_;  // save dvl velocity


  // buffer to hold some usbl sets in the beginning in order to initalize 
  std::vector<Sophus::Vector6d> initializer_buffer_;
  std::string dock_frame_id_;
  
  // Filter Algorithm object
  DockingFilter docking_filter_;

};
