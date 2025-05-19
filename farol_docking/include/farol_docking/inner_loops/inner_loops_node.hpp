/**
 * @file   inner_loops_node.hpp
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
#include <farol_docking/Reference3.h>

// farol libraries
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <farol_docking/utils/docking_utils.hpp>
#include <farol_docking/utils/logging_utils.hpp>
#include <farol_docking/inner_loops/smc.hpp>
#include <farol_docking/inner_loops/pid.hpp>


/**
 * @brief  Interface between ROS and the docking controller algorithm
 */
 class InnerLoopNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   /**
    * @brief Sets up ROS params publishers, subscribers and timer
    *
    * @param[in] nodehandle
    * @param[in] nodehandle_private
    */
 	InnerLoopNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /**
   * @brief  Destructor
   */
 	~InnerLoopNode();

 private:

  /**
   * @brief Callback to State messages from the nav_filter and the docking_filter
   * @param msg state message
   *
   */
  void state_callback(const auv_msgs::NavigationStatus &msg);

  /**
   * @brief  Timer iteration callback
   *
   * @Param event
   */
  void position_ref_callback(const farol_docking::Reference3 &msg);
  void attitude_ref_callback(const farol_docking::Reference3 &msg);

  /**
   * @brief  Timer iteration callback
   *
   * @Param event
   */
  void flag_callback(const std_msgs::Int8 &msg);

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
  ros::Subscriber sub_filter_state_;
  ros::Subscriber sub_docking_state_;
  ros::Subscriber sub_position_ref_;
  ros::Subscriber sub_attitude_ref_;
  ros::Subscriber sub_flag_;

 	// Publishers
  ros::Publisher force_request_pub_;
  ros::Publisher debug_pub_;

  // ROS messages
  std_msgs::Float64 ref_msg_;
  std_msgs::Int8 flag_msg_;
  auv_msgs::BodyForceRequest force_request_msg_;

  // Timer
  ros::Timer timer_;

  // ROS Parameters
  double p_node_frequency_;
  bool debug_;
  std::string controller_type_;
  std::string reference_frame_;


 	// Problem variables ˇˇˇˇ
  bool start_;
  int flag_;

  double last_it_time_;

  double t_position_ref_, t_attitude_ref_;

  std::array<bool, 6> disable_axis_; 

  std::unique_ptr<ControllerBase> controller_;

};
