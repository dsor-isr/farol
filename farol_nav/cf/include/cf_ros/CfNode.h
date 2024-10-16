/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_CFNODE_H
 #define CATKIN_WS_CFNODE_H

 //some generically useful stuff to include...
 #include <std_msgs/String.h>
 #include <vector>
 #include <ros/ros.h> 

 #include <farol_gimmicks_library/FarolGimmicks.h>
 #include <dsor_msgs/Measurement.h>
 #include <auv_msgs/NavigationStatus.h>
 #include <farol_msgs/mState.h>
 #include <Eigen/Core>
 #include <cf/Tuning.h>

/* -------------------------------------------------------------------------*/
/**
 * @brief  ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 */
/* -------------------------------------------------------------------------*/
 class CfNode {
 public:
   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	CfNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~CfNode();

  

  // @.@ Public methods



 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

 	// @.@ Subscribers
  ros::Subscriber sub_reset_;
  ros::Subscriber sub_tuning_;
  ros::Subscriber sub_estimator_;
  ros::Subscriber sub_no_measures_;

  ros::Subscriber sub_position_;
  ros::Subscriber sub_velocity_;
  ros::Subscriber sub_orientation_;

 	// @.@ Publishers
  ros::Publisher state_pub_;
  ros::Publisher State_pub_;
  ros::Publisher meas_pub_;

 	// @.@ Timer
 	ros::Timer timer_;           ///< ROS Timer

  // @.@ Parameters from Yaml
  double node_frequency_;   ///< node frequency

 	// @.@ Problem variables

  Eigen::Matrix4d identity4_;   // Auxiliary matrices
  Eigen::Matrix2d identity2_;

  Eigen::Vector4d state_;
  Eigen::Matrix4d process_v_;
  Eigen::MatrixXd input_;
  Eigen::Vector2d velocity_;
  double yaw_;

  Eigen::Vector2d total_velocity_;

  // sensors
  std::vector<std::string> sensor_list_{"gnss","usbl","usbl_top","dvl_bt","ahrs"};

  std::vector<dsor_msgs::Measurement> measurements_;

  ros::Time kf_time_;

  double last_K_time_;

  double wn_, csi_;
  Eigen::Vector2d error_;
  Eigen::Vector2d measure_est_;
  bool no_measures_;

  bool estimator_;

  Eigen::Vector4d last_state_;

  Eigen::Vector2d last_dvl_, last_gnss_, last_usbl_;
  double last_ahrs_;
  double outlier_dvl_{10}, outlier_gnss_{5}, outlier_usbl_{5}, outlier_ahrs_{20};

  double outlier_tolerance_usbl_top_;
  int reject_counter_usbl_top_{0}, reject_counter_max_usbl_top_;
  bool wait_first_pos_meas_{true};

  // ros::Time delta_t;

  // @.@ Encapsulation the gory details of initializing subscribers, publishers and services
 	


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
  void measurementCallback(const dsor_msgs::Measurement &msg);
  void tuningCallback(const cf::Tuning &msg);
  void estimatorCallback(const std_msgs::Empty &msg);
  void nomeasuresCallback(const std_msgs::Empty &msg);
  void stateCallback(const auv_msgs::NavigationStatus &msg);
  void resetCallback(const std_msgs::Empty &msg);

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
  void estimation(double delta_t);
  void correction(double delta_t);
  void initialReset(double easting, double northing);

};
#endif //CATKIN_WS_CONTROLNODE_H
