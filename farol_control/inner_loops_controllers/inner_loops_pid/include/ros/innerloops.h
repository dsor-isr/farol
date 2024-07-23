#ifndef MDS_INNERLOOPS_H
#define MDS_INNERLOOPS_H

#include "ros_controller.h"
#include <algorithm>
#include <ros/ros.h>

#include "inner_loops_pid/ChangeFFGains.h"
#include "inner_loops_pid/ChangeInnerGains.h"
#include "inner_loops_pid/ChangeInnerLimits.h"
#include <auv_msgs/BodyForceRequest.h>
#include <auv_msgs/NavigationStatus.h>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

/**
 * @brief  Implementation of the inner loops. Computes the forces and torques
 * (tau) to be applied on the vehicle based on desired reference values.
 */
class Innerloops {
public:
  /**
   * @brief  Contructor of the innerloops class
   *
   * @param nh  ROS nodehandle to subscribe, publish and read parameters.
   */
  Innerloops(ros::NodeHandle &nh);

  /**
   * @brief  Destructor of the innerloops class.
   */
  ~Innerloops();

private:
  /**
   * @brief  Function to initialize subscribers
   */
  void initializeSubscribers();
  /**
   * @brief Function to initialize services
   *
   */
  void initializeServices();

  /**
   * @brief  Function to initialize publishers
   */
  void initializePublishers();

  /**
   * @brief  Function to initialize timer function
   */
  void initializeTimer();

  /**
   * @brief  Reads the frequency the timer function will run at
   *
   * @return  Node frequency
   */
  double nodeFrequency();

  /**
   * @brief  Main loop of the innerloops. Computes the forces and torques
   * necessary and publishes to the respective topic
   *
   * @param event
   */
  void timerCallback(const ros::TimerEvent &event);

  /**
   * @brief  Reads the state topic and saves into the respective variables
   *
   * @param msg  State message
   */
  void StateCallback(const auv_msgs::NavigationStatus &msg);

  /**
   * @brief  Receives a desired force to be applied to the vehicle that should
   * be added to the output of the innerloops
   *
   * @param msg Desired force
   */
  void forceBypassCallback(const auv_msgs::BodyForceRequest &msg);

  /**
   * @brief  Turn radius speed from either surge or airmar measurement (delfim)
   *
   * @param msg Float (surge/airmar speed)
   */
  void turnRadiusSpeedCallback(const auv_msgs::NavigationStatus &msg);

  /**
   * @brief Service to change feedforward gains
   *
   * @param req client request
   * @param res server response
   * @return true
   * @return false
   */
  bool changeFFGainsService(inner_loops_pid::ChangeFFGains::Request &req,
                          inner_loops_pid::ChangeFFGains::Response &res);

  /**
   * @brief Service to change gains
   *
   * @param req client request
   * @param res server response
   * @return true
   * @return false
   */
  bool changeGainsService(inner_loops_pid::ChangeInnerGains::Request &req,
                          inner_loops_pid::ChangeInnerGains::Response &res);
  /**
   * @brief Service to change Limits
   *
   * @param req client request
   * @param res server response
   * @return true
   * @return false
   */
  bool
  changeLimitsService(inner_loops_pid::ChangeInnerLimits::Request &req,
                      inner_loops_pid::ChangeInnerLimits::Response &res);

  /**
   * @brief Service to turn on and off turning radius limiter
   *
   * @param req client request
   * @param res server response
   * @return true
   * @return false
   */
  bool
  turningRadiusLimiterService(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

  // Handlers
  ros::NodeHandle nh_;
  std::vector<RosController *> controllers_;

  // State Variables
  double yaw_, pitch_, roll_, yaw_rate_, pitch_rate_, roll_rate_;
  double surge_, sway_, heave_;
  double depth_, altitude_, vdepth_, valtitude_;

  double force_request_[3]{}; // Forces, x,y,z (body)

  double torque_request_[3]{}; // Torques, x,y,z (body)

  // Boolean to store whether we use soft by-pass of body forces or hard by-pass
  // Soft - sums the output of the forces given by the PIDs and the ones received from external topics
  // Hard - discards the values from the PIDs and only uses the ones received from external topics
  bool forces_hard_bypass_;
  double timeout_ref_;

  ros::Subscriber st_sub_; // State subscriber
  ros::Subscriber force_bypass_sub_;
  ros::Subscriber turn_radius_speed_sub_; // Airmar surge speed subscriber (Delfim)

  ros::ServiceServer change_ff_gains_srv_;
  ros::ServiceServer change_gains_srv_;
  ros::ServiceServer change_limits_srv_;
  ros::ServiceServer turning_radius_limiter_; // Service for turning radius limiter

  ros::Timer timer_; // timer

  double zero = 0.0;

  // variable to bypass innerloops and publish a direct force on the vehicle
  auv_msgs::BodyForceRequest force_bypass_;
  // force bypass receiving time
  ros::Time ref_force_bypass_;

  // Forces and Torques publisher
  ros::Publisher ft_pub_;

  // Turn Radius Flag Publisher
  ros::Publisher turn_radius_flag_pub_;
  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_;

  // Turning Radius Limiter (2D) Variables
  bool turn_radius_flag_{false};
  RateLimiter rate_limiter_;
  double turn_radius_speed_;
  double turn_radius_speed_t_;
};

#endif // MDS_INNERLOOPS_H
