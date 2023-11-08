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
  Eigen::VectorXd init_value_;

  Eigen::MatrixXd predict_cov_;
  Eigen::MatrixXd update_cov_;

  Eigen::VectorXd state_;
  Eigen::MatrixXd process_;
  Eigen::MatrixXd input_;
                    
  Eigen::MatrixXd process_cov_;
  Eigen::MatrixXd complementary_cov_;

  Eigen::_cov_;

  double originLat_{38.765852};
  double originLon_{-9.09281873};

  // sensors
  std::vector<std::string> sensor_list_{"gps","usbl","dvl_bt","ahrs"};
 	std::vector<struct sensor_config> sensors_;

  std::vector<Eigen::VectorXd> measurements_;

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
  void predict();
  void update();

};
#endif //CATKIN_WS_CONTROLNODE_H
