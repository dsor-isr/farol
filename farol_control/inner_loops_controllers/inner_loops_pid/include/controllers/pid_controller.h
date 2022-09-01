#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <dsor_utils/filters/lowpass_filter.hpp>
#include <farol_msgs/mPidDebug.h>

/**
 * @brief  Implementation of a PID with anti windup
 */
class PID_Controller {
public:
  // Disable controller
  bool disable;

  /**
   * @brief  Constructor of a pid controller with kp, ki and kd equal to 0
   */
  PID_Controller();

  /**
   * @brief  Constructor of a PID controller. Initially enabled.
   *
   * @param Kp Proporcional gain
   * @param Ki Integral gain
   * @param Kd Derivative gain
   * @param max_error maximum reference error allowed
   * @param max_out minimum reference error allowed
   */
  PID_Controller(float Kp, float Ki, float Kd, float max_error, float max_out);

  /**
   * @brief  Constructor of a PID controller. Initially enabled.
   *
   * @param Kp Proporcional gain
   * @param Ki Integral gain
   * @param Kd Derivative gain
   * @param max_error maximum reference error allowed
   * @param max_out maximum output allowed
   * @param min_error minimum refrence error allowed
   * @param min_out minimum output allowed
   */
  PID_Controller(float Kp, float Ki, float Kd, float max_error, float max_out,
                 float min_error, float min_out);

  /**
   * @brief  Constructor of a PID controller (without low pass filter). Initially enabled.
   *
   * @param Kp Proporcional gain
   * @param Ki Integral gain
   * @param Kd Derivative gain
   * @param Kff Feedforward gain
   * @param Kff_d Feedforward gain (linear drag) []
   * @param Kff_dd Feedforward gain (quadratic drag)
   * @param max_error maximum reference error allowed
   * @param max_out maximum output allowed
   * @param min_error minimum refrence error allowed
   * @param min_out minimum output allowed
   */
  PID_Controller(float Kp, float Ki, float Kd, float Kff, float Kff_d, float Kff_lin_drag, float Kff_quad_drag,
                 float max_error, float max_out, float min_error, float min_out);

  /**
   * @brief  Constructor of a PID controller (with low pass filter). Initially enabled.
   *
   * @param Kp Proporcional gain
   * @param Ki Integral gain
   * @param Kd Derivative gain
   * @param Kff Feedforward gain
   * @param Kff_d Feedforward gain (linear drag) []
   * @param Kff_dd Feedforward gain (quadratic drag)
   * @param max_error maximum reference error allowed
   * @param max_out maximum output allowed
   * @param min_error minimum refrence error allowed
   * @param min_out minimum output allowed
   * @param lpf_dt Low pass filter sampling time
   * @param lpf_fc Low pass filter cutoff param
   */
  PID_Controller(float Kp, float Ki, float Kd, float Kff, float Kff_d, float Kff_lin_drag, float Kff_quad_drag,
                 float max_error, float max_out, float min_error, float min_out, double lpf_dt, double lpf_fc);

  /**
   * @brief Core function. Computes the output of the PID.
   *
   * @param error_p Error between the reference and the estimated variable
   * @param ref_value Reference value to compute the feedforward term
   * @param duration Sample time
   * @param debug Check if we want to generate a debugging message to later publish
   *
   * @return
   */
  float computeCommand(float error_p, float ref_value, float duration, bool debug);

  /**
   * @brief  Reset function. Sets the integral error term to 0.
   */
  void reset();

  /**
   * @brief Set the feedfoward Gains object
   *
   * @param ff_gain Feefoward gain of reference
   * @param ff_d_gain Feedoforward gain of reference derivative 
   * @param ff_lin_drag_gain Feefoward gain (for linear drag)
   * @param ff_quad_drag_gain Feefoward gain (for quadratic drag)
   */
  void setFFGains(const float &ff_gain, const float &ff_d_gain, const float &ff_lin_drag_gain, const float &ff_quad_drag_gain); 

  /**
   * @brief Set the Gains object
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void setGains(const float &kp, const float &ki, const float &kd);

  /**
   * @brief Set the Limit Bounds object
   *
   * @param max_out maximum output allowed
   * @param min_out minimum output allowed
   */
  void setLimitBounds(const float &max_out, const float &min_out);

  /**
   * @brief Get the Gains object
   *
   * @return std::vector<float> const
   */
  std::vector<double> getGains() const;

  /**
   * @brief Get the Limit Bounds object
   *
   * @return std::vector<float> const
   */
  std::vector<double> getLimitBounds() const;

  /**
   * @brief Get debug info from the PID controller internal variables
   *
   * @return std::vector<float> const
   */
  farol_msgs::mPidDebug getDebugInfo() const;

protected:
  // Controller PID Gains
  float p_gain_, i_gain_, d_gain_;
  // Controller feedforwad gains
  float ff_gain_, ff_d_gain_, ff_lin_drag_gain_, ff_quad_drag_gain_;
  // Max and Min output/error values
  float max_error_, max_out_, min_error_, min_out_;
  // Integral error
  float integral_;
  // Previous error and reference value
  float pre_error_, prev_ref_value_;
  
  // Low pass filter object
  bool has_lpf_{false};
  std::unique_ptr<LowPassFilter> lpf_; 

  // Debug message
  farol_msgs::mPidDebug msg_debug_;

private:

  /**
   * @brief  Saturation function. Clips a variable based on upper and lower
   * boundaries
   *
   * @param u Variable to be saturated
   * @param low lower boundary
   * @param high upper boundary
   *
   * @return
   */
  float sat(float u, float low, float high);
};

#endif // PID_CONTROLLER_H
