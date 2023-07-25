#ifndef CATKIN_WS_PINGERNODE_H
#define CATKIN_WS_PINGERNODE_H

#include <cmath>
#include <stdlib.h>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <dmac/DMACPayload.h>
#include <farol_msgs/mUSBLFix.h>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <interrogation_scheme/StartSilent.h>

//#include <comms_srv/KeyValueSrv.h>

#define SOUND_SPEED 1500.0

// using dmac::mUSBLFix;

/* -------------------------------------------------------------------------*/
/**
 * @brief  
 */
/* -------------------------------------------------------------------------*/
class SilentPinger
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
  SilentPinger(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);
  ~SilentPinger();

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
   * @brief  Get the last modem estimate
   *
   * @Returns   
   */
  /* -------------------------------------------------------------------------*/
  double getTimeBeforeStart();

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Get the last modem estimate
   *
   * @Returns   
   */
  /* -------------------------------------------------------------------------*/
  void startCallback(const interrogation_scheme::StartSilent& msg);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Get the last modem estimate
   *
   * @Returns   
   */
  /* -------------------------------------------------------------------------*/
  void timeFromAnchorCallback(const std_msgs::UInt32& msg);

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
  void pingNextNode(double tinit_aux = 0);

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
  ros::Subscriber sub_in_modem, sub_serializer, sub_enable, sub_tinit;

  //-----------------------------------------------------------
  // ROS Publishers
  ros::Publisher pub_im, pub_range, pub_triggerserialization, pub_deserialization, pub_tinit;

  //-----------------------------------------------------------
  // ROS Timer
  ros::Timer timer;

  //-----------------------------------------------------------
  // Parameters
  std::vector<std::string> modems_list; // Vector with the number of the modems to ping
  std::vector<double> slot_list;
  double tslack;

  //-----------------------------------------------------------
  // Variables
  std::queue<int> modems;
  std::queue<double> slots;
  double tlastRECVIMS;
  unsigned long int tslot_start;
  unsigned long int tslot_end;
  double range_max;
  bool waiting_for_serializer;
  bool ENABLE_;
  bool slot_ack;
  int modem_id;
  bool init;
  int anchor;
  int ok_check;
  unsigned long int tinit;
  int tnew_start;
  bool first_time;

  // Time variables
  unsigned long int lastRECVTime_modem;
  ros::Time lastRECVTime_ros;

  void loadParams();
  void initializeSubscribers();
  void initializePublishers();
  void initializeTimer();


};
#endif