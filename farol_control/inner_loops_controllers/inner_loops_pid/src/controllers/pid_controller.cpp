#include "pid_controller.h"

PID_Controller::PID_Controller() : p_gain_(0), i_gain_(0), d_gain_(0) {
  reset();
  disable = true;
}

PID_Controller::PID_Controller(float Kp, float Ki, float Kd, float max_error,
                               float max_out)
    : p_gain_(Kp), i_gain_(Ki), d_gain_(Kd), max_error_(max_error), max_out_(max_out),
      min_error_(-max_error), min_out_(-max_out) {
  reset();
  disable = true;
}

PID_Controller::PID_Controller(float Kp, float Ki, float Kd, float max_error,
                               float max_out, float min_error, float min_out)
    : p_gain_(Kp), i_gain_(Ki), d_gain_(Kd), max_error_(max_error), max_out_(max_out), min_error_(min_error), min_out_(min_out) {
  reset();
  disable = true;
}

PID_Controller::PID_Controller(float Kp, float Ki, float Kd, float Kff, float Kff_d, float Kff_lin_drag, float Kff_quad_drag,  
                               float max_error, float max_out, float min_error, float min_out)
      : p_gain_(Kp), i_gain_(Ki), d_gain_(Kd), ff_gain_(Kff), ff_d_gain_(Kff_d), ff_lin_drag_gain_(Kff_lin_drag), ff_quad_drag_gain_(Kff_quad_drag),
        max_error_(max_error), max_out_(max_out), min_error_(min_error), min_out_(min_out) {
  reset();
  disable = true;

  yaw_rate_prev = 0;
  yaw_prev = 0;
  g_filter_prev = 0;
  u_prev = 0;
  u_sat_prev = 0;
  first_it = true;
}

PID_Controller::PID_Controller(float Kp, float Ki, float Kd, float Kff, float Kff_d, float Kff_lin_drag, float Kff_quad_drag,  
                               float max_error, float max_out, float min_error, float min_out, double lpf_dt, double lpf_fc) 
      : p_gain_(Kp), i_gain_(Ki), d_gain_(Kd), ff_gain_(Kff), ff_d_gain_(Kff_d), ff_lin_drag_gain_(Kff_lin_drag), ff_quad_drag_gain_(Kff_quad_drag),
        max_error_(max_error), min_error_(min_error), max_out_(max_out), min_out_(min_out) {
  reset();
  disable = true;
  has_lpf_ = true;
  lpf_ = std::make_unique<LowPassFilter>(lpf_dt, 2*M_PI*lpf_fc);
}

// Edu
float PID_Controller::computeCommandYaw(float yaw, float yaw_rate, float yaw_ref, float duration, float frequency) {
  // Initialisation parameters (must be moved to an init function of this node or ros_controller node (preferred))
  N_r = -0.5;
  I_z = 0.24;
  u_max = 5.0; // N.m
  u_min = -5.0; // N.m
  a = 10.0; // rad/s  
  alpha = 1.0 / I_z;
  beta = -N_r / I_z;
  w_n = 0.5; // rad/s
  qsi = 0.7;
  pole = -4;

  // delta = duration;
  delta = 1.0 / frequency;

  K_r = (2*qsi*w_n - pole - beta) / alpha;
  K_p = w_n * (w_n - 2*pole*qsi) / alpha;
  K_i = (- pow(w_n, 2) * pole) / alpha;
  k_a = 1.0 / delta;

  A = std::exp(-a*delta);
  B = 1 - A;

  // change from degrees to radians
  yaw = yaw / 360 * 2*M_PI;
  yaw_rate = yaw_rate / 360 * 2*M_PI;
  yaw_ref = yaw_ref / 360 * 2*M_PI;

  // Compute control input
  error = yaw_ref - yaw;

  if (error > M_PI) {
    error = error - 2*M_PI;
  } else if (error < - M_PI) {
    error = error + 2*M_PI;
  }

  if (first_it) {
    yaw_rate_dot = 0;
    yaw_dot = 0;
  } else {
    yaw_rate_dot = (yaw_rate - yaw_rate_prev) / delta;
    yaw_dot = (yaw - yaw_prev); // still needs to be divided by delta

    if (yaw_dot > M_PI) {
      yaw_dot = yaw_dot - 2*M_PI;
    } else if (yaw_dot < - M_PI) {
      yaw_dot = yaw_dot + 2*M_PI;
    }

    yaw_dot = yaw_dot / delta;
  }

  g = K_r * yaw_rate_dot + K_p * yaw_dot;

  g_filter = A*g_filter_prev + g*B;
  // g_filter = g;

  u_d = K_i * error - g_filter;
  u_dot = u_d - k_a * (u_prev - u_sat_prev);
  double u;
  u = u_prev + u_dot * delta;
  
  if (u < u_min) {
    u_sat = u_min;
  } else if (u > u_max) {
    u_sat = u_max;
  } else {
    u_sat = u;
  }

  // Update new prev values
  yaw_rate_prev = yaw_rate;
  yaw_prev = yaw;
  g_filter_prev = g_filter;
  u_prev = u;
  u_sat_prev = u_sat;

  first_it = false;

  // return output
  return u_sat;
}

// Ravi bad -> convert to vertical
float PID_Controller::computeCommandAltitude(float altitude, float altitude_rate, float altitude_ref, float duration, float frequency) {

  // PID Controller Gains
  // double Zw = -4.1879;
  // double Zww = -40.9649;  
  ROS_WARN_STREAM("Low pass filter cutoff frequency must be higher than 0.");
  
  double mw = 29.9081;
  double dw = -1.1130;
  double ksi = 0.7;
  double w0 = 0.5;
  double p = 10 * ksi * w0;

  double a = 10;
  double A = std::exp(-a * duration);
  double B = 1 - A;

  K_p = mw * (2 * ksi * w0 * p + w0 * w0);
  K_i = mw * w0 * w0 * p;
  K_d = mw * (2 * ksi * w0 + p) - dw;
  K_a = 1 / duration;

  u_max = 40.0; // N.m
  u_min = -40.0; // N.-m

  Dt = duration;
  // Dt = 1.0 / frequency;
  
  ROS_INFO_STREAM("ALtitude: "<<altitude<<"| Altitude_rate: "<< altitude_rate);
  // Compute control input
  error = altitude_ref - altitude;

  // compute derivative of altitude_rate  
  if (first_it)
    h_dot_dot = 0;
  else 
    h_dot_dot = (altitude_rate - altitude_rate_prev) / Dt;
  
  h_dot_dot_filter = A*h_dot_dot_filter_prev + h_dot_dot*B;
  
  // add all terms
  g = K_d * h_dot_dot_filter + K_p * altitude_rate;
  u_d = K_i * error - g;
  ROS_INFO_STREAM("u_d: "<<u_d);

  // integrate with anti wind-up
  u_dot = u_d - K_a * (u_prev - u_sat_prev);
  double u;
  u = u_prev + u_dot * duration;
  ROS_INFO_STREAM("u: "<<u);

  if (u < u_min) {
    u_sat = u_min;
  } else if (u > u_max) {
    u_sat = u_max;
  } else {
    u_sat = u;
  }
  ROS_INFO_STREAM("u_sat: "<<u_sat);

  // Update new prev values
  altitude_rate_prev = altitude_rate;
  h_dot_dot_filter_prev = h_dot_dot_filter;
  g_filter_prev = g_filter;
  u_prev = u;
  u_sat_prev = u_sat;
  
  first_it = false;

  // return output
  return -u_sat;
}

// Ravi good
float PID_Controller::computeCommandSpeed(float speed, float speed_ref, float Dt, bool debug) {

  // Don't return nothing if controller is disabled
  if (disable || Dt < 0.05 || Dt > 0.2)
    return 0.0;

  // // filter reference signal through low pass if it exists
  // if (has_lpf_) {
  //   ref_d_value = lpf_->update((ref_value - ref_prev_) / Dt);
  // }
  // else {
  //   ref_d_value = (ref_value - ref_prev_) / Dt;
  // }
  float error = speed_ref- speed;
  float error_sat = sat(speed_ref - speed , min_error_, max_error_);

  // compute derivative of speed by 1st order aproximation  
  float speed_dot;
  //ROS_INFO_STREAM("ff_gain_:" << ff_gain_);
  if (first_it){
    speed_dot = 0;
    u_prev_ = ff_gain_;
    u_sat_prev_ = u_prev_;
  } // actually ff_gain_ is gw, starting the integrator at gw is good
  else {
    speed_dot = (speed-speed_prev_)/Dt;//(speed-speed_prev_)>0.05 ? (speed-speed_prev_)/Dt : 0;
  }
  //ROS_INFO_STREAM("speed_dot:" << speed_dot);

  // aply a low pass filter because previous computation amplifies noise
  double speed_dot_filter;
  double a = 31.4;                      // pole of the low pass filter
  double lpf_A = std::exp(-a * Dt);   // descretization of the filter
  double lpf_B = 1 - A;               // descretization of the filter
  speed_dot_filter = lpf_A*speed_dot_filter_prev_ + speed_dot*lpf_B;
  //ROS_INFO_STREAM("speed_dot_filter:" << speed_dot_filter);

  // // LPF from chatgpt
  // double speed_dot_filter;
  // double a = 10;                      // pole of the low pass filter
  // double alpha = (2 * a) / (Dt + a);
  // double beta  = (Dt - a) / (Dt + a);
  // speed_dot_filter = alpha*(speed - speed_prev_) - beta*speed_dot_filter_prev_;
  // ROS_INFO_STREAM("speed_dot_filter:" << speed_dot_filter);

  // value of the ouput before integration with anti-windup
  
  double tau_d;
  tau_d =  i_gain_ * error - p_gain_ * speed_dot;
  //ROS_INFO_STREAM("tau_d:" << tau_d);

  // integration with anti windup
  double K_a = 1/Dt;
  u_dot = tau_d - K_a * (u_prev_ - u_sat_prev_);
  //ROS_INFO_STREAM("u_dot:" << u_dot);
  double u;
  u = u_prev_ + u_dot * Dt;
  //ROS_INFO_STREAM("u:" << u);
  // aply the saturation
  if (u < min_out_) {
    u_sat = min_out_;
  } else if (u > max_out_) {
    u_sat = max_out_;
  } else {
    u_sat = u;
  }
  //ROS_INFO_STREAM("u_sat:" << u_sat);


  if (true) {
    msg_debug_.ref = speed_ref;
    msg_debug_.ref_d = (speed_ref - ref_prev_) / Dt;
    if (has_lpf_) {
      msg_debug_.ref_d_filtered = 0;
    } else {
      msg_debug_.ref_d_filtered = (speed_ref - ref_prev_) / Dt;
    }
    msg_debug_.error = error;
    msg_debug_.error_saturated = error_sat;

    msg_debug_.ffTerm = speed_dot;
    msg_debug_.ffDTerm = speed_dot_filter;
    msg_debug_.ffDragTerm = tau_d;
    msg_debug_.pTerm = u_dot;
    msg_debug_.iTerm = 0;
    msg_debug_.dTerm = 0;
    msg_debug_.output = 0;
  }

  // update previous values
  speed_prev_ = speed;
  speed_dot_prev_ = speed_dot;
  speed_dot_filter_prev_ = speed_dot_filter;
  
  u_prev_ = u;
  u_sat_prev_ = u_sat;
  ref_prev_ = speed_ref;
  first_it=false;
  
  return u_sat;
}

//Ravi good
float PID_Controller::computeCommandAttitude(float attitude, float attitude_rate, float attitude_ref, float Dt, bool debug) {
  
  if (disable || Dt < 0.05 || Dt > 0.2)
    return 0.0;
  
  // // filter reference signal through low pass if it exists
  // if (has_lpf_) {
  //   ref_d_value = lpf_->update((ref_value - prev_ref_value_) / Dt);
  // }
  // else {
  //   ref_d_value = (ref_value - prev_ref_value_) / Dt;
  // }
  
  // convert degrees to radians
  attitude = attitude / 180*M_PI;
  attitude_rate = attitude_rate / 180*M_PI;
  attitude_ref = attitude_ref / 180*M_PI;

  // Compute control input
  float error = wrapToPi(attitude_ref- attitude);
  float error_sat = attitude_ref- attitude;//sat(error, min_error_, max_error_);
  
  // if first iteration dont copmute derivative
  float attitude_rate_dot, attitude_dot, attitude_rate_dot_filter;
  if (first_it) {
    attitude_rate_dot = 0;
    attitude_dot = 0;
  } else {
    attitude_rate_dot = (attitude_rate - attitude_rate_prev_) / Dt;
    // aply a low pass filter because previous computation amplifies noise
    double a = 31.4;                      // pole of the low pass filter
    double lpf_A = std::exp(-a * Dt);   // descretization of the filter
    double lpf_B = 1 - A;               // descretization of the filter
    attitude_rate_dot_filter = lpf_A*attitude_rate_dot_filter_prev_ + attitude_rate_dot*lpf_B;

    //attitude_dot = wrapToPi(attitude - attitude_prev)/Dt;
    attitude_dot = attitude_rate;
  }

  // adding up all PID terms
  double tau_d;
  tau_d =  i_gain_ * error - p_gain_ * attitude_dot - d_gain_*attitude_rate_dot_filter;

  // integration with anti windup
  double K_a = 1/Dt;
  u_dot = tau_d - K_a * (u_prev_ - u_sat_prev_);
  double u;
  u = u_prev_ + u_dot * Dt;

  // aply the saturation
  double u_sat;
  if (u < min_out_) {
    u_sat = min_out_;
  } else if (u > max_out_) {
    u_sat = max_out_;
  } else {
    u_sat = u;
  }

  if (true) {
    msg_debug_.ref = attitude_ref;
    msg_debug_.ref_d = (attitude_ref - ref_prev_) / Dt;
    if (has_lpf_) {
      msg_debug_.ref_d_filtered = 0;
    } else {
      msg_debug_.ref_d_filtered = (attitude_ref - ref_prev_) / Dt;
    }
    msg_debug_.error = error;
    msg_debug_.error_saturated = error_sat;

    msg_debug_.ffTerm = attitude_dot;
    msg_debug_.ffDTerm = attitude_rate_dot_filter;
    msg_debug_.ffDragTerm = tau_d;
    msg_debug_.pTerm = p_gain_;
    msg_debug_.iTerm = i_gain_;
    msg_debug_.dTerm = d_gain_;
    msg_debug_.output = 0;
  }

  // Update prev values
  attitude_rate_prev_ = attitude_rate;
  attitude_prev_ = attitude;
  attitude_rate_dot_filter_prev_ = attitude_rate_dot_filter;

  u_prev_ = u;
  u_sat_prev_ = u_sat;
  ref_prev_ = attitude_ref;
  first_it=false;

  // return output
  return u_sat;
}

// Default controller
float PID_Controller::computeCommand(float error_p, float ref_value, float duration, bool debug) {
  float ref_d_value;

  // Don't return nothing if controller is disabled
  if (disable || duration < 0.05 || duration > 0.2)
    return 0.0;

  // filter reference signal through low pass if it exists
  if (has_lpf_) {
    ref_d_value = lpf_->update((ref_value - prev_ref_value_) / duration);
  }
  else {
    ref_d_value = (ref_value - prev_ref_value_) / duration;
  }

  float current_value = error_p + ref_value;
  float error = sat(error_p, min_error_, max_error_);
  integral_ += error * duration;

  // Compute PID Terms
  float ffTerm = ff_gain_ * std::abs(ref_value) * ref_value;
  float ffDTerm = ff_d_gain_ * ref_d_value;
  float ffDragTerm = ( ff_lin_drag_gain_ + ff_quad_drag_gain_ * std::abs(current_value) ) * current_value;
  float pTerm = p_gain_ * error;
  float iTerm = i_gain_ * integral_;
  float dTerm = d_gain_ * (error - pre_error_) / duration;

  float out = ffTerm + ffDTerm + ffDragTerm + pTerm + iTerm + dTerm;

  // Saturate output
  if (out > max_out_) {
    integral_ -= error * duration;
  } else if (out < min_out_) {
    integral_ -= error * duration;
  }

  if (debug) {
    msg_debug_.ref = ref_value;
    msg_debug_.ref_d = (ref_value - prev_ref_value_) / duration;
    if (has_lpf_) {
      msg_debug_.ref_d_filtered = ref_d_value;
    } else {
      msg_debug_.ref_d_filtered = (ref_value - prev_ref_value_) / duration;
    }
    msg_debug_.error = error_p;
    msg_debug_.error_saturated = error;
    msg_debug_.ffTerm = ffTerm;
    msg_debug_.ffDTerm = ffDTerm;
    msg_debug_.ffDragTerm = ffDragTerm;
    msg_debug_.pTerm = pTerm;
    msg_debug_.iTerm = iTerm;
    msg_debug_.dTerm = dTerm;
    msg_debug_.output = out;
  }

  // Saturate output
  if (out > max_out_) {
    out = max_out_;
  } else if (out < min_out_) {
    out = min_out_;
  }

  pre_error_ = error;
  prev_ref_value_ = ref_value;

  return out;
}


void PID_Controller::reset() {
  // General controllers
  integral_ = 0;
  pre_error_ = 0;
  prev_ref_value_ = 0;
  // my controllers
  u_prev_=ff_gain_;
  u_sat_prev_=ff_gain_;
}

void PID_Controller::setFFGains(const float &kff, const float &kff_d, const float &kff_lin_drag,
                              const float &kff_quad_drag) {
  ff_gain_ = kff;
  ff_d_gain_ = kff_d;
  ff_lin_drag_gain_ = kff_lin_drag;
  ff_quad_drag_gain_ = kff_quad_drag;
}

void PID_Controller::setGains(const float &kp, const float &ki,
                              const float &kd) {
  p_gain_ = kp;
  i_gain_ = ki;
  d_gain_ = kd;
}

void PID_Controller::setLimitBounds(const float &max_out,
                                    const float &min_out) {
  max_out_ = max_out;
  min_out_ = min_out;
}

std::vector<double> PID_Controller::getGains() const {
  return std::vector<double>{p_gain_, i_gain_, d_gain_};
}

std::vector<double> PID_Controller::getLimitBounds() const {
  return std::vector<double>{max_out_, min_out_};
}

farol_msgs::mPidDebug PID_Controller::getDebugInfo() const {
  return msg_debug_;
}

float PID_Controller::sat(float u, float low, float high) {
  if (u < low) return low;
  if (u > high) return high;
  return u;
}

float PID_Controller::wrapToPi(float angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

