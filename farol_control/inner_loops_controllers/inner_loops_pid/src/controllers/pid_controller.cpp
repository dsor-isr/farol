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
  integral_ = 0;
  pre_error_ = 0;
  prev_ref_value_ = 0;
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
