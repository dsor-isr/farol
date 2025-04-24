/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_DOCKINGCONTROLLERNODE_H
 #define CATKIN_WS_DOCKINGCONTROLLERNODE_H

 // ROS stuff
 #include <ros/ros.h> 
 #include <std_msgs/String.h>
 #include <std_msgs/Empty.h>
 #include <std_msgs/Int8.h>
 #include <std_msgs/Float64.h>
 #include <geometry_msgs/PoseStamped.h>  
 #include <dsor_msgs/Measurement.h>
 #include <farol_docking/ControllerDebug.h>
 #include <auv_msgs/NavigationStatus.h>
 #include <farol_docking/dState.h>


 // Other libraries
 #include <vector>
 #include <cmath> 
 #include <Eigen/Eigen>
 #include <farol_gimmicks_library/FarolGimmicks.h>
 #include "DockingController.h"
 #include "docking_utils.h"

/* -------------------------------------------------------------------------*/
/**
 * @brief  ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 */
/* -------------------------------------------------------------------------*/
 class DockingControllerNode {
 public:
   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	DockingControllerNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~DockingControllerNode();

  

  // @.@ Public methods



 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

 	// @.@ Subsctibers
  ros::Subscriber sub_state_;
  ros::Subscriber sub_velocity_;
  ros::Subscriber sub_orientation_;
  ros::Subscriber sub_start_;
  ros::Subscriber sub_flag_;
  ros::Subscriber sub_dock_inertial_pos_;
  ros::Subscriber sub_filter_state_;
  

 	// @.@ Publishers
  ros::Publisher surge_ref_pub_;
  ros::Publisher sway_ref_pub_;
  ros::Publisher yaw_ref_pub_;
  ros::Publisher depth_ref_pub_;
  ros::Publisher flag_pub_;
  ros::Publisher debug_pub_;
  

 	// @.@ Timer
 	ros::Timer timer_;           ///< ROS Timer

  // @.@ Parameters from Yaml
  double p_node_frequency_;   ///< node frequency

 	// @.@ Problem variables
  Eigen::Vector2d dvl_;
  double ahrs_;
  bool start_;
  int flag_;
  bool debug_;
  std::vector<double> params_;
  
  DockingController controller_;

  std_msgs::Float64 ref_msg_;
  std_msgs::Int8 flag_msg_;
  farol_docking::ControllerDebug debug_msg_;

  ros::Time last_update_time_;
  Eigen::Vector4d dock_pose_;
  Eigen::Vector2d filter_state_;
  ros::Time end_time_;
  bool reached_close_{false};

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
  void state_callback(const farol_docking::dState &msg);
  void measurement_callback(const dsor_msgs::Measurement &msg);
  void start_callback(const std_msgs::Empty &msg);
  void flag_callback(const std_msgs::Int8 &msg);
  void dock_pose_callback(const farol_msgs::mState &msg);
  void filter_state_callback(const auv_msgs::NavigationStatus &msg);



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


};
#endif //CATKIN_WS_CONTROLNODE_H
