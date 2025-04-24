/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_CFNODE_H
 #define CATKIN_WS_CFNODE_H

 //some generically useful stuff to include...
 #include <ros/ros.h> 
 #include <std_msgs/String.h>
 #include <geometry_msgs/PoseStamped.h>  
 #include <geometry_msgs/Point.h>  
 #include <dsor_msgs/Measurement.h>
 #include <farol_msgs/mState.h>
 #include <farol_msgs/mUSBLFix.h>
 #include <auv_msgs/NavigationStatus.h>
 #include <farol_docking/FilterDebug.h>
 #include <farol_docking/dState.h>

 #include <vector>
 #include <bitset>
 #include <algorithm>
 #include <farol_gimmicks_library/FarolGimmicks.h>
 #include <Eigen/Core>
 #include <sophus/se3.hpp>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_ros/transform_broadcaster.h>
 #include "tf2_ros/message_filter.h"

 #include "DockingHFilter.h"
 #include "DockingRFilter.h"
 #include "DockingVFilter.h"
 #include "docking_utils.h"

/* -------------------------------------------------------------------------*/
/**
 * @brief  ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 */
/* -------------------------------------------------------------------------*/
 class DockingFilterNode {
 public:
   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	DockingFilterNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~DockingFilterNode();

  

 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

 	// @.@ Subscribers
  ros::Subscriber sub_reset_;
  ros::Subscriber sub_velocity_;
  ros::Subscriber sub_orientation_;
  ros::Subscriber sub_position_;
  ros::Subscriber sub_usbl_fix_;
  ros::Subscriber sub_usbl_accoms_;
  ros::Subscriber sub_dock_inertial_pos_;
  

 	// @.@ Publishers
  ros::Publisher state_pub_;
  ros::Publisher console_state_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher debug2_pub_;

 	// @.@ Timer
 	ros::Timer timer_;    

  // @.@ ROS Parameters
  double node_frequency_; 

  // ROS messages
  farol_docking::dState state_msg_;
  farol_msgs::mState console_state_msg_;
  farol_docking::FilterDebug debug_msg_;

  // @.@ Measurements stuff
  std::bitset<3> usbl_state_;
  Sophus::SE3d observations_;
  double prev_yaw_;
  double prev_yaw_time_;
  double prev_depth_;
  double prev_depth_time_;
  bool yaw_derivative_{false};
  std::vector<Eigen::Vector4d> initializer_measurements_;
  std::vector<Eigen::Vector2d> dvl_outlier_window_;
  const size_t WINDOW_SIZE = 20;

  Eigen::Vector3d usbl_auv_;
  Eigen::Vector3d usbl_dock_;

  Eigen::Vector4d dock_pose_;
  
  // @.@ Filters
  DockingRFilter r_filter_;
  DockingHFilter h_filter_;
  DockingVFilter v_filter_;

  // @.@ Other stuff
  double time_last_compass_;
  double last_compass_;
  bool new_compass_{false};
  Eigen::Vector2d inertial_pos_;
  bool dvl_outlier_rejection_{false};
  bool usbl_outlier_rejection_{false};
  bool debug_{false};

  geometry_msgs::Point usbl_debug_msg;



  /* -------------------------------------------------------------------------*/
  /**
   * @brief Load parameters from parameter server 
   */
  /* -------------------------------------------------------------------------*/
 	void loadParams();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Initialize ROS node Subscribers
   */
  /* -------------------------------------------------------------------------*/
  void initializeSubscribers();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Initialize ROS node Publishers
   */
  /* -------------------------------------------------------------------------*/
 	void initializePublishers();

 
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Initialize ROS node Services
   */
  /* -------------------------------------------------------------------------*/
 	void initializeServices();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Initialize ROS node Timer  
   */
  /* -------------------------------------------------------------------------*/
 	void initializeTimer();


 	
  // @.@ Callbacks declaration
  void measurement_callback(const dsor_msgs::Measurement &msg);
  void usbl_callback(const farol_msgs::mUSBLFix &msg);
  void reset_callback(const std_msgs::Empty &msg);
  void dock_pose_callback(const farol_msgs::mState &msg);
  bool isOutlier(const Eigen::Vector2d& measurement);


  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Timer iteration callback
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerIterCallback(const ros::TimerEvent& event);


  // @.@ Services declaration



  // @.@ Member helper functions
  Eigen::Vector4d extract_observations();
  void estimation(double delta_t);
  void correction(double delta_t);

};

#endif //CATKIN_WS_CONTROLNODE_H
