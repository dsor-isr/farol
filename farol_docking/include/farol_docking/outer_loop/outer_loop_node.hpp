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
#include <optional>

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
#include <waypoint/sendWpType1.h>

// farol libraries
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <farol_docking/utils/docking_utils.hpp>  


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
  void usbl_callback(const farol_msgs::mUSBLFix &msg);
  void force_callback(const auv_msgs::BodyForceRequest &msg);


  void check_state_transition();
  void generate_refs(double time_now, double Dt);

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
  ros::Subscriber sub_force_;

 	// Publishers
  ros::Publisher surge_ref_pub_;
  ros::Publisher sway_ref_pub_;
  ros::Publisher yaw_ref_pub_;
  ros::Publisher depth_ref_pub_;
  ros::Publisher attitude_pub_;
  ros::Publisher position_pub_;
  ros::Publisher force_request_pub_;
  ros::Publisher flag_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher force_pub_;
  ros::Publisher docking_state_pub;

  // Services
  ros::ServiceClient wp_client;
  
  // ROS interfaces
  std_msgs::Float64 ref_msg_;
  std_msgs::Int8 flag_msg_;
  farol_docking::Reference3 ref_3d_msg_;
  auv_msgs::BodyForceRequest   force_request_msg_;
  waypoint::sendWpType1 wp_srv_;
  std_msgs::String phase_msg_;

  // #farol_docking::ControllerDebug debug_msg_;
  
 	// Timer
 	ros::Timer timer_;

  // ROS Parameters
  double node_frequency_;

  int flag_{0};
  std::string state_{"idle"};
  
  // A priori information on the positionof the dock
  Eigen::Vector2d dock_position_;
  std::optional<double> dock_depth_, dock_altitude_, dock_heading_, safe_depth_approach_;
  
  double new_time_, last_update_time_, Dt_;
  bool first_it_;
  
  Eigen::Vector2d filter_state_;
  ros::Time end_time_;
  bool reached_close_{false};
  bool waiting_completion_{false};
  bool got_docking_state_{false};
  double time_last_acomms_{-1.0};
  double acomms_timeout_{20.0}, homing_dist_, acomms_search_radius_;
  int acomms_n_min_fix_;
  int n_fixes_;
  
  // for generating trajectory
  double y_ref_{0.0}, y_ref_dot_{0.0}, y_ref_ddot_{0.0};
  double z_ref_{0.0}, z_ref_dot_{0.0}, z_ref_ddot_{0.0};
  double x_ref_{0.0}, x_ref_dot_{0.0}, x_ref_ddot_{0.0};
  double yaw_ref_{0.0}, yaw_ref_dot_{0.0}, yaw_ref_ddot_{0.0};
  double homing_initial_time_;
  double homing_converging_time_y_, homing_converging_time_z_, homing_converging_time_x_;
  double homing_initial_y_, homing_initial_z_, homing_initial_x_;
  double u_terminal_;
  double prev_yaw_ref_dot_;

  Eigen::Vector3d max_accl_;
  
  Eigen::Vector3d docking_state_, inertial_state_; 
  
  Eigen::Vector2d homing_target_point_;

};
