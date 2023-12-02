/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_CKFNODE_H
 #define CATKIN_WS_CKFNODE_H

 //some generically useful stuff to include...
 #include <std_msgs/String.h>
 #include <vector>
 #include <ros/ros.h> 

 #include <farol_gimmicks_library/FarolGimmicks.h>
 #include <dsor_msgs/Measurement.h>
 #include <auv_msgs/NavigationStatus.h>
 #include <Eigen/Core>

/* -------------------------------------------------------------------------*/
/**
 * @brief  ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 */
/* -------------------------------------------------------------------------*/
 class CkfNode {
 public:


  struct sensor_config{
    std::string name;
    std::vector<int> config;    // this field let's us know what totake from the measurement message
    Eigen::MatrixXd noise;
    double outlier_tolerance, outlier_increase;
    int reject_counter;
  };

   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	CkfNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~CkfNode();

  

  // @.@ Public methods



 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

 	// @.@ Subscribers
  ros::Subscriber sub_reset_;
  ros::Subscriber sub_position_;
  ros::Subscriber sub_velocity_;
  ros::Subscriber sub_orientation_;

 	// @.@ Publishers
  ros::Publisher state_pub_;

 	// @.@ Timer
 	ros::Timer timer_;           ///< ROS Timer

  // @.@ Parameters from Yaml
  double node_frequency_;   ///< node frequency
  
  // std::vector<std::string> sensor_list_;
  // std::vector<struct sensor_config> sensor_configurations;

 	// @.@ Problem variables
  Eigen::Matrix4d predict_cov_;
  Eigen::Matrix4d update_cov_;

  Eigen::Vector4d state_;
  Eigen::Matrix4d process_;
  Eigen::Matrix4d process_v_;
  Eigen::MatrixXd input_;
  Eigen::Vector2d velocity_;
                    
  Eigen::Matrix4d process_cov_;
  Eigen::Matrix4d complementary_cov_;

  // Eigen::_cov_;

  double originLat_{38.765852};
  double originLon_{-9.09281873};

  // sensors
  std::vector<std::string> sensor_list_{"gnss","usbl","dvl_bt"};
 	std::vector<struct sensor_config> sensors_;

  std::vector<dsor_msgs::Measurement> measurements_;

  ros::Time kf_time_;

  Eigen::Vector2d last_dvl_, last_gnss_, last_usbl_;
  double outlier_dvl_{10}, outlier_gnss_{10}, outlier_usbl_{10};

  auv_msgs::NavigationStatus state_msg_;
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
  void predict(double delta_t);
  void update(double delta_t);

};
#endif //CATKIN_WS_CONTROLNODE_H
