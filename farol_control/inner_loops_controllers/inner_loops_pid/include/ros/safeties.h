#ifndef SAFETIES_H
#define SAFETIES_H

#include <auv_msgs/NavigationStatus.h>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

class Safeties {
private:
  double min_altitude_{0.0}; // minimum altitude (safety)
  double state_depth_{0.0}; // vehicle's depth value
  double state_altitude_{0.0}; // vehicle's altitude value

  // Subscribers
  ros::Subscriber depth_sub_; 
  ros::Subscriber altitude_sub_;
  ros::Subscriber state_sub_;
  // Publishers
  ros::Publisher depth_safety_pub_;
  ros::Publisher altitude_safety_pub_;

public:
  /**
   * @brief Innerloops safeties constructor
   *
   * @param nh ros nodehandle to subscribe and publish topics
   */
  Safeties(ros::NodeHandle &nh);

  /**
   * @brief  Innerloops safeties Destructor
   */
  virtual ~Safeties();

  /**
   * @brief  Method to read parameters from yaml files
   *
   * @param nh ros nodehandle to subscribe and publish topics
   */
  void loadParams(ros::NodeHandle &nh);

  /**
   * @brief  Method to initialize subscribers
   *
   * @param nh ros nodehandle to subscribe and publish topics
   */
  void initializeSubscribers(ros::NodeHandle &nh);

  /**
   * @brief  Method to initialize publishers
   *
   * @param nh ros nodehandle to subscribe and publish topics
   */
  void initializePublishers(ros::NodeHandle &nh);

  /**
   * @brief  Method called when a depth reference is received. Will check
   * safeties regarding altitude:
   * if valid will publish in the depth controller topic the desired reference,
   * otherwise will publish the minimum altitude in the altitude controller.
   *
   * @param msg Desired depth
   */
  void depthSafetyCallback(const std_msgs::Float64 &msg);

  /**
   * @brief  Method called when a altitude reference is received. Will check
   * safeties regarding altitude:
   * if above the minimum altitude will publish the desired reference, otherwise
   * will publish the minimum altitude
   *
   * @param msg Desired altitude
   */
  void altitudeSafetyCallback(const std_msgs::Float64 &msg);

  /**
   * @brief  Method called when a state msg is received. Updates depth and
   * altitude values to calculate the water collumn needed for depth safeties.
   *
   * @param msg Vehicles state
   */
  void stateCallback(const auv_msgs::NavigationStatus &msg);
};

#endif /* SAFETIES_H */
