#ifndef __ROS_CONTROLLER__
#define __ROS_CONTROLLER__

#include "pid_controller.h"
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <farol_msgs/mPidDebug.h>
#include <dsor_utils/filters/rate_limiter.hpp>
#include <dsor_utils/rotations.hpp>
#include <dsor_utils/math.hpp>



/**
 * @brief  ROS implementation of the innerloops controllers. Based on a desired reference computes the force or torque to be applied.
 */
class RosController {
public:
  /**
   * @brief  Constructor of a innerloop controller.
   *
   * @param nh  ROS nodehandle to read parameters and subscribe to relevant
   * topics
   * @param controller_name  Controller name (variable being controlled)
   * @param refCallback_topic  Topic name
   * @param state  Pointer to state variable being controlled
   * @param force_or_torque  Pointer to force or torque output
   * @param frequency Frequency of controller sampling rate
   */
  RosController(ros::NodeHandle &nh, std::string controller_name,
                std::string refCallback_topic, double *state,
                double *force_or_torque, double frequency);

  /**
   * @brief  Constructor of a innerloop controller
   *
   * @param nh  ROS nodehandle to read parameters and subscribe to relevant
   * topics
   * @param controller_name  Controller name (variable being controlled)
   * @param refCallback_topic  Topic name
   * @param state  Pointer to state variable being controlled
   * @param state_dot  Pointer to the derivative of the state variable being
   * controlled
   * @param force_or_torque  Pointer to force or torque output
   * @param frequency Frequency of controller sampling rate
   */
  RosController(ros::NodeHandle &nh, std::string controller_name,
                std::string refCallback_topic, double *state, double *state_dot,
                double *force_or_torque, double frequency);

/**
   * @brief  Constructor of a innerloop yaw controller.
   *
   * @param nh  ROS nodehandle to read parameters and subscribe to relevant
   * topics
   * @param controller_name  Controller name (variable being controlled)
   * @param refCallback_topic  Topic name
   * @param state  Pointer to state variable being controlled
   * @param force_or_torque  Pointer to force or torque output
   * @param frequency Frequency of controller sampling rate
   * @param turn_limiter_flag On/off flag for turning radius limiter
   * @param surge Surge speed of vehicle
   * @param rate_limiter Rate Limiter object from dsor_utils
   */
  RosController(ros::NodeHandle &nh, std::string controller_name,
                std::string refCallback_topic, double *state, double *yaw_rate,
                double *force_or_torque, double frequency,
                bool *turn_limiter_flag, double *turn_radius_speed, RateLimiter *rate_limiter, double *turn_radius_speed_t);

  /**
   * @brief  Constructor of a innerloop yaw_rate controller.
   *
   * @param nh  ROS nodehandle to read parameters and subscribe to relevant
   * topics
   * @param controller_name  Controller name (variable being controlled)
   * @param refCallback_topic  Topic name
   * @param state  Pointer to state variable being controlled
   * @param force_or_torque  Pointer to force or torque output
   * @param frequency Frequency of controller sampling rate
   * @param turn_limiter_flag On/off flag for turning radius limiter
   * @param surge Surge speed of vehicle
   */
  RosController(ros::NodeHandle &nh, std::string controller_name,
                std::string refCallback_topic, double *state,
                double *force_or_torque, double frequency,
                bool *turn_limiter_flag, double *turn_radius_speed, double *turn_radius_speed_t);

  /**
   * @brief  Core function. Computes the PID output
   *
   * @return  The force or torque that result from the PID computation
   */
  virtual double computeCommand();

  /**
   * @brief  Setter function for the circular units flag
   *
   * @param flag true for controllers using angles, false otherwise
   */
  void setCircularUnits(const bool &flag) { circular_units_ = flag; }
 
  /**
   * @brief Set the Positive Output object
   * 
   * @param flag 
   */
  void setPositiveOutput(const bool &flag) { positive_output_ = flag; }

  /**
   * @brief Get the Controller Name object
   * 
   */
  std::string getControllerName() const {return controller_name_;}

  /**
   * @brief Set the Feedforwar gains P I D object
   * 
   * @param kff Feedforward gain (quadratic)
   * @param kff_d Feedforwad derivative gain
   * @param kff_lin_drag Feedforward linear drag gain 
   * @param kff_quad_drag Feedforwad quadratic drag gain
   */
  void setFFGainsPID(const float &kff, const float &kff_d, const float &kff_lin_drag, const float kff_quad_drag) {pid_c_->setFFGains(kff, kff_d, kff_lin_drag, kff_quad_drag);}

  /**
   * @brief Set the Gains P I D object
   * 
   * @param kp Proportional gain 
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void setGainsPID(const float &kp, const float &ki, const float &kd) {pid_c_->setGains(kp, ki, kd);}

  void setLimitBoundsPID(const float &max_out, const float &min_out) { pid_c_->setLimitBounds(max_out, min_out);}

protected:
  /**
   * @brief  Callback function. Saturates the value if boundaries exist
   *
   * @param msg  Float64 value of the variable being controlled
   */
  void refCallback(const std_msgs::Float64 &ptr);

  /**
   * @brief  Initialize function. Reads the parameters, creates a pid controller
   * if any gain is different from zero and subscribes to the relevant topic.
   *
   * @param nh  Nodehandle to read parameters and subscribe to relevant
   * topics
   * @param controller_name  Controller name (variable being controlled)
   * @param refCallback_topic  Topic name
   */
  void init(ros::NodeHandle &nh, std::string controller_name,
            std::string refCallback_topic);
  /**
   * @brief  Check if the reference is new
   *
   * @return  True if valid, false otherwise
   */
  virtual bool validRef();

  ros::Duration timeout_ref_;   // inverval of time where a reference is valid
  std::string controller_name_; // string of the variable being controlled (for
                                // reading parameters purpose)
  double ref_value_;     // reference value of the variable being controlled
  double max_ref_value_; // maximum value of the reference being controlled
  double min_ref_value_; // minimum value of the reference being controlled
  ros::Time ref_time_;   // timestamp of the reference
  ros::Time last_cmd_;   // last controller call
  bool debug_;           // flag to check wheter to output or not pid internal information 
  farol_msgs::mPidDebug debug_msg_; // msg to publish debug information

  // pointers to state values
  double *state_ptr_;

  // pointer to correspondig force or torque
  double *force_or_torque_ptr_;

  // frequency of the controller sampling rate
  double frequency_;

  // pointer to flag for turning ON/OFF the turn limiter for yaw and yaw_rate
  bool *turn_limiter_flag_ptr_;

  double min_turn_radius_; // maxuimum turning radius to saturate yaw  rate reference
  bool turn_limit_yaw_ref_{false};
  bool turn_limit_yaw_rate_ref_{false};
  double *turn_radius_speed_;
  double *turn_radius_speed_t_;
  double turn_radius_speed_t_max_;
  
  // respective handler to the slew rate limiter class
  RateLimiter *rate_limiter_ptr_;  

  bool circular_units_;  // for angles
  bool positive_output_; // positive output or switch sign

  // handlers
  PID_Controller *pid_c_;
  ros::Subscriber ros_sub_;
  ros::Publisher debug_pub_;

  double *yaw_rate_ptr_;
  double *altitude_rate_ptr_;
};

#endif /* ifndef __ROS_CONTROLLER__*/
