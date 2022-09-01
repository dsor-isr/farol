#ifndef CATKIN_WS_PINGERNODE_H
#define CATKIN_WS_PINGERNODE_H

#include <cmath>
#include <stdlib.h>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <dmac/DMACPayload.h>
#include <farol_msgs/mUSBLFix.h>
#include <farol_gimmicks_library/FarolGimmicks.h>

//#include <comms_srv/KeyValueSrv.h>

#define SOUND_SPEED 1500.0

// using dmac::mUSBLFix;

/* -------------------------------------------------------------------------*/
/**
 * @brief  
 */
/* -------------------------------------------------------------------------*/
class AcousticPinger
{
public:
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param nodehandle
   * @Param nodehandle_private
   */
  /* -------------------------------------------------------------------------*/
  AcousticPinger(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);
  ~AcousticPinger();

  // TODO: Ask for Clock to the Modem
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param tmodem
   * @Param tros
   */
  /* -------------------------------------------------------------------------*/
  void updateClock(unsigned long int tmodem, ros::Time tros);
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Get the last modem estimate
   *
   * @Returns   
   */
  /* -------------------------------------------------------------------------*/
  unsigned long int getModemClockNow();

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
  void triggerSerialization();
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   */
  /* -------------------------------------------------------------------------*/
  void pingNextNode();

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
  void serializerCallback(const std_msgs::String& msg);
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
  void EnableCallback(const std_msgs::Bool& msg);
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
  void RECVIMSCallback(const dmac::DMACPayload& msg);
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param e
   */
  /* -------------------------------------------------------------------------*/
  void Timer(const ros::TimerEvent& e);
  
private:

  //-----------------------------------------------------------
  // ROS Node Variable
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_;
  

  //-----------------------------------------------------------
  // ROS Subscribers
  ros::Subscriber sub_in_modem, sub_serializer, sub_enable;

  //-----------------------------------------------------------
  // ROS Publishers
  ros::Publisher pub_im, pub_range, pub_triggerserialization, pub_deserialization;

  //-----------------------------------------------------------
  // ROS Timer
  ros::Timer timer;

  //-----------------------------------------------------------
  // Parameters
  std::vector<std::string> modems_list; // Vector with the number of the modems to ping
  std::vector<double> timeout_list;
  double tslack;

  //-----------------------------------------------------------
  // Variables
  std::queue<int> modems;
  std::queue<double> timeouts;
  unsigned long int tlastRECVIMS;
  unsigned long int tlastSENDIMS;
  double range_max;
  bool waiting_for_serializer;
  bool ENABLE_;

  // Time variables
  unsigned long int lastRECVTime_modem;
  ros::Time lastRECVTime_ros;

  void loadParams();
  void initializeSubscribers();
  void initializePublishers();
  void initializeTimer();


};
#endif