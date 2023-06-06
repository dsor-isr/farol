/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico */
 #ifndef CATKIN_WS_REPLIERNODE_H
 #define CATKIN_WS_REPLIERNODE_H

 //some generically useful stuff to include...
 #include <cmath>
 #include <stdlib.h>
 #include <queue>
 #include <map>

 #include <ros/ros.h> //ALWAYS need to include this 

 // ROS messages and stuff
 // Examples
 #include <std_msgs/Empty.h>
 #include <std_msgs/String.h>
 #include <dmac/DMACPayload.h>
 #include <farol_msgs/mUSBLFix.h>
 #include <farol_gimmicks_library/FarolGimmicks.h>

 #include <ctime>

 #define SOUND_SPEED 1513.0
 #define SLACK_TIME 1000000.0

/* -------------------------------------------------------------------------*/
/**
 * @brief  
 */
/* -------------------------------------------------------------------------*/
 class ReplierNode {
 public:
   /* -------------------------------------------------------------------------*/
   /**
    * @brief  Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	ReplierNode(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~ReplierNode();

 private:
 	// put private member data here; "private" data will only be available to member functions of this class;
 	ros::NodeHandle nh_, nh_private_; // we will need this, to pass between "main" and constructor

 	// some objects to support subscriber and publishers, these will be set up within the class constructor, hiding the ugly details

 	// @.@ Subsctibers
 	ros::Subscriber ack_sub_;
	ros::Subscriber serializer_sub_;

 	// @.@ Publishers
 	ros::Publisher range_pub_;
	ros::Publisher im_pub_;
	ros::Publisher trigger_serialization_pub_;
	ros::Publisher deserialization_pub_;

 	// @.@ Timer
 	ros::Timer timer_;

 	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
 	// member variables will retain their values even as callbacks come and go

 	// +.+ Parameters from Yaml
	int p_modem_id_;
	double p_tslack_;
  std::vector<double> timeout_list;
 	
 	// +.+ Problem variables
 	double timeout_, range_;
  std::queue<double> timeouts;
	std::map<int, unsigned long int> t_last_recv_ims_;
	unsigned long int t_last_send_ims_;
	int modem_id_last_send_ims_;
	dmac::DMACPayload reply_ims_;
  bool pinger_ack;
  double range_max;

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
 	void initializeSubscribers();
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
 	void initializePublishers();
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
 	void initializeTimer();
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
 	void loadParams();

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
	void triggerSerialization();

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
 	void timerIterCallback(const ros::TimerEvent& event);
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
	void replyCallback(const dmac::DMACPayload& msg);
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
	void serializerCallback(const std_msgs::String& msg);
	
	
};
#endif //CATKIN_WS_ReplierNode_H
