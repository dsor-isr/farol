/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico */
#ifndef CATKIN_WS_WAYPOINTNODE_H
#define CATKIN_WS_WAYPOINTNODE_H

// some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

// ROS messages and stuff
// ROS messages and stuff
#include "waypoint/sendWpType1.h"
#include <auv_msgs/NavigationStatus.h>
#include <geometry_msgs/PointStamped.h>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <wp_heading.h>
#include <wp_loose.h>
#include <wp_standard.h>

/**
 * @brief  ROS node class
 */
class WaypointNode {
public:
  WaypointNode(ros::NodeHandle *nodehandle,
                 ros::NodeHandle *nodehanle_private);

  ~WaypointNode();

private:
  WaypointController *wp_controller_; ///< Pointer to waypoint controller
  WPref_t wp_ref_;                    ///< Desired waypoint
  Vehicle_t veh_state_;               ///< Vehicle state
  double node_frequency_;             ///< node main loop frequency

  // Nodehandles
  ros::NodeHandle nh_;   ///< Public ros nodehandle
  ros::NodeHandle nh_p_; ///< Private ros nodehandle

  // Subscribers
  ros::Subscriber flag_sub_;  ///< flag subscriber
  ros::Subscriber state_sub_; ///< state subscriber
  ros::Subscriber turn_radius_flag_sub_; ///< turn radius limiter flag subscriber

  // Publishers
  ros::Publisher u_ref_pub_;        ///< surge publisher
  ros::Publisher yaw_ref_pub_;      ///< sway publisher
  ros::Publisher flag_pub_;         ///< flag publisher
  ros::Publisher v_ref_pub_;        ///< sway publisher
  ros::Publisher yaw_rate_ref_pub_; ///< yaw rate publisher

  // Services
  ros::ServiceServer wp_standard_srv_; ///< standard waypoint service
  ros::ServiceServer wp_loose_srv_;    ///< loose waypoint service
  ros::ServiceServer wp_heading_srv_;  ///< heading waypoint service

  // Timers
  ros::Timer timer_; ///< main loop timer

  // Parameters from Yaml
  double cdist_;

  // Type 1 (Standard or Loose)
  double ku_;
  double ks_;
  double delta_t_;
  double speed_turn_;

  // Type 2 (Heading)
  double k1_;
  double k2_;
  double k3_;

  // Topic names
  std::string state_topic_;
  std::string flag_topic_;
  std::string turn_radius_flag_topic_;
  std::string wp_ref_topic_;
  std::string u_ref_topic_;
  std::string v_ref_topic_;
  std::string yaw_ref_topic_;
  std::string yaw_rate_ref_topic_;
  std::string wp_standard_topic_;
  std::string wp_loose_topic_;
  std::string wp_heading_topic_;

  // Turn Radius Flag Boolean
  bool turn_radius_flag_{false};

  /**
   * @brief  Function to initialize subscribers
   */
  void initializeSubscribers();

  /**
   * @brief  Function to initialize publishers
   */
  void initializePublishers();

  /**
   * @brief  Function to initialize services
   */
  void initializeServices();

  /**
   * @brief  Function to initialize the node main loop timer
   */
  void initializeTimer();

  /**
   * @brief  Loads parameters from parameter server
   */
  void loadParams();

  /**
   * @brief  Reads the node main loop frequency from parameter service
   *
   * @return  main loop frequency
   */
  double nodeFrequency();

  /**
   * @brief  Callback function of the state topic.
   * Updates the vehicle state object to be then used in the waypoint
   * controllers
   *
   * @param msg contains the state info
   */
  void updateCallback(const auv_msgs::NavigationStatus &msg);

  /**
   * @brief  Callback function of the flag topic.
   * Stops the main loop if flag is different from 4 (4 equals WP mode)
   *
   * @param msg contains the flag info
   */
  void flagCallback(const std_msgs::Int8 &msg);

  /**
   * @brief  Callback function of the turn radius flag topic.
   *
   * @param msg contains the flag info
   */
  void turnRadiusFlagCallback(const std_msgs::Bool &msg);

  /**
   * @brief  Main loop of the node. Executes the waypoint controller and
   * publishes the respective references
   *
   * @param event
   */
  void timerCallback(const ros::TimerEvent &event);

  /**
   * @brief  Callback function of standard waypoint service. Starts main loop
   *
   * @param req
   * @param res
   *
   * @return
   */
  bool sendWpStandardService(waypoint::sendWpType1::Request &req,
                             waypoint::sendWpType1::Response &res);
  /**
   * @brief  Callback function of loose waypoint service. Starts main loop
   *
   * @param req
   * @param res
   *
   * @return
   */
  bool sendWpLooseService(waypoint::sendWpType1::Request &req,
                          waypoint::sendWpType1::Response &res);

  /**
   * @brief  Callback function of waypoint with heading control service. Starts
   * main loop
   *
   * @param req
   * @param res
   *
   * @return
   */
  bool sendWpHeadingService(waypoint::sendWpType1::Request &req,
                            waypoint::sendWpType1::Response &res);

  /**
   * @brief  Substitutes the waypoint controller pointer
   *
   * @param new_wp
   */
  void createWaypoint(WaypointController *new_wp);

  bool decodeWaypoint(double x, double y);
};
#endif // CATKIN_WS_WAYPOINTNODE_H
