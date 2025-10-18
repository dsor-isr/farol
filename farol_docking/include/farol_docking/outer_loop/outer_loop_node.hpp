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
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <auv_msgs/NavigationStatus.h>
#include <auv_msgs/BodyForceRequest.h>
#include <dsor_msgs/Measurement.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/mUSBLFix.h>
#include <farol_docking/Reference3.h>
#include <farol_docking/SE3Ref.h>
#include <farol_docking/SetGain.h>
#include <waypoint/sendWpType1.h>

// farol libraries
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <farol_docking/utils/docking_utils.hpp>  
#include <farol_docking/outer_loop/trajectory.hpp>  


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


  void check_state_transition(double time_now);
  void plan_trajectory();
  bool eval_trajectory(double time_now);

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
  ros::Publisher yaw_ref_pub_,altitude_ref_pub_;
  ros::Publisher depth_ref_pub_;
  ros::Publisher se3_ref_pub_;
  ros::Publisher force_request_pub_;
  ros::Publisher flag_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher docking_state_pub;
  ros::Publisher mission_string_pub;

  // Services
  ros::ServiceClient wp_client;
  
  // ROS interfaces
  std_msgs::Float64 ref_msg_;
  std_msgs::Int8 flag_msg_;
  farol_docking::SE3Ref se3_ref_msg_;
  auv_msgs::BodyForceRequest   force_request_msg_;
  waypoint::sendWpType1 wp_srv_;
  std_msgs::String phase_msg_;
  std_msgs::String mission_string_msg_;


  ros::ServiceServer reconfig_srv_;
  bool reconfigureParamSrv(farol_docking::SetGain::Request& req,
                           farol_docking::SetGain::Response& res);

  // #farol_docking::ControllerDebug debug_msg_;
  
 	// Timer
 	ros::Timer timer_;

  // ROS Parameters
  double node_frequency_;

  int flag_{0};
  std::string state_{"idle"};
  
  // A priori information on the positionof the dock
  Eigen::Vector2d dock_position_;
  std::optional<double> dock_depth_, dock_altitude_, dock_heading_, safe_depth_approach_,safe_altitude_approach_;
  
  double new_time_, last_update_time_, Dt_;
  bool first_it_;
  
  bool got_docking_state_{false};   // signals if docking state has been received yet
  double time_last_acomms_{-1.0};   // time since last got usbl fix 
  double acomms_timeout_{20.0};     // timeout to reset manoeuvre if no fixes are received
  double acomms_search_radius_;     // radius to search for acoms menoeuvre
  double terminal_dist_;
  double terminal_thrust_;
  double aproach_dist_;            // distace ahead of dock opening to go to
  double acomms_n_min_fix_;
  
  // For generating trajectory
  TrajectoryPlanner trajectory_;  // trajectory object
  bool traj_planned_ = false;     // flag to signal if trajectory has been planned 
  double homing_dist_{2.5};            // distace ahead of dock opening to go to
  double homing_initial_time_;    // time at which homing sequence started
  double u_terminal_{0.05};             // velocity at which to go in to dock 
  double v_max_u_{0.2};  // [m/s]
  double v_max_v_{0.2};  // [m/s]
  double a_max_t_{0.5};  // [m/s²]
  double w_max_{0.2};    // [m/s]
  double a_w_max_{0.5};  // [m/s²]
  double r_max_{0.5};    // [rad/s]
  double a_r_max_{1.0};  // [rad/s²]
  double jerk_ratio_{1.0};  // [rad/s²]
  tf2::Quaternion q_aux_;


  // to save the vehicle states docking and intertial 
  Eigen::Vector3d docking_state_, inertial_state_; 
  Eigen::Vector4d terminal_force_;
  double inertial_yaw_, docking_yaw_;
  
  // i dont remember what this is for
  Eigen::Vector2d homing_target_point_;




  // double y_ref_{0.0}, y_ref_dot_{0.0}, y_ref_ddot_{0.0};
  // double z_ref_{0.0}, z_ref_dot_{0.0}, z_ref_ddot_{0.0};
  // double x_ref_{0.0}, x_ref_dot_{0.0}, x_ref_ddot_{0.0};
  // double yaw_ref_{0.0}, yaw_ref_dot_{0.0}, yaw_ref_ddot_{0.0};
  // double homing_converging_time_y_, homing_converging_time_z_, homing_converging_time_x_;
  // double homing_initial_y_, homing_initial_z_, homing_initial_x_;
  // double prev_yaw_ref_dot_;
};
