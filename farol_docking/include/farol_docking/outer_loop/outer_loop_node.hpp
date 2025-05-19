/**
 * @file   docking_controller_node.hpp
 * @brief  Docking Controller Node class
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

// ros libraries
#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>  
#include <geometry_msgs/Point.h>  
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <auv_msgs/NavigationStatus.h>
#include <auv_msgs/BodyForceRequest.h>
#include <dsor_msgs/Measurement.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/mUSBLFix.h>

// farol libraries
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <farol_docking/utils/docking_utils.hpp>  
#include <farol_docking/outer_loop/outer_loop.hpp>  


/**
 * @brief  Interface between ROS and the docking controller algorithm
 */
 class OuterLoopNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   /**
    * @brief Constructor
    *
    * @param[in] nodehandle
    * @param[in] nodehandle_private
    */
 	OuterLoopNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /**
   * @brief  Destructor
   */
 	~OuterLoopNode();

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
   * @brief Callback to State messages from the nav_filter and the docking_filter
   * @param msg state message
   * 
   */
  void state_callback(const auv_msgs::NavigationStatus &msg);


  void measurement_callback(const dsor_msgs::Measurement &msg);
  void start_callback(const std_msgs::Empty &msg);
  void flag_callback(const std_msgs::Int8 &msg);
  void dock_pose_callback(const farol_msgs::mState &msg);
  void filter_state_callback(const auv_msgs::NavigationStatus &msg);

  /**
   * @brief  Timer iteration callback
   *
   * @Param event
   */
  void timerIterCallback(const ros::TimerEvent& event);


  // ROS node handlers
 	ros::NodeHandle nh_;
 	ros::NodeHandle nh_private_;  

 	// Subscribers
  ros::Subscriber sub_inertial_state_;
  ros::Subscriber sub_docking_state_;
  ros::Subscriber sub_velocity_;
  ros::Subscriber sub_start_;
  ros::Subscriber sub_flag_;

 	// Publishers
  ros::Publisher surge_ref_pub_;
  ros::Publisher sway_ref_pub_;
  ros::Publisher i_yaw_ref_pub_;
  ros::Publisher i_depth_ref_pub_;
  ros::Publisher d_attitude_pub_;
  ros::Publisher d_position_pub_;
  ros::Publisher force_request_pub_;
  ros::Publisher flag_pub_;
  ros::Publisher debug_pub_;
  
  // ROS messages
  std_msgs::Float64 ref_msg_;
  std_msgs::Int8 flag_msg_;
  // geometry_msgs::Quaternion d_attitude_ref_msg;
  geometry_msgs::Vector3 d_attitude_ref_msg_;
  geometry_msgs::Vector3 d_position_ref_msg_;
  auv_msgs::BodyForceRequest   force_request_msg_;
  // #farol_docking::ControllerDebug debug_msg_;
  
 	// Timer
 	ros::Timer timer_;

  // ROS Parameters
  double p_node_frequency_;
  double debug_;

 	// Problem variables ˇˇˇˇ
  bool start_;
  int flag_;
  bool debug_;

  std::vector<double> params_;
  ros::Time last_update_time_;
  Eigen::Vector2d filter_state_;
  ros::Time end_time_;
  bool reached_close_{false};
  
  // Docking Controller Algorithm Object
  DockingController docking_controller_;

};
