// smc.cpp
#include <farol_docking/inner_loops/pid.hpp>  

PID::PID(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private) : ControllerBase(nodehandle, nodehandle_private) {
  // Parameters
  x_pid.p_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/x/kp", 10);
  x_pid.i_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/x/ki", 1);
  x_pid.d_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/x/kd", 1);
  x_pid.min_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/x/min_out", -10);
  x_pid.max_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/x/max_out", 10);
  x_pid.controller_name_ = "x";

  y_pid.p_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/y/kp", 10);
  y_pid.i_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/y/ki", 1);
  y_pid.d_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/y/kd", 1);
  y_pid.min_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/y/min_out", -10);
  y_pid.max_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/y/max_out", 10);
  y_pid.controller_name_ = "y";

  z_pid.p_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/z/kp", 10);
  z_pid.i_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/z/ki", 1);
  z_pid.d_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/z/kd", 1);
  z_pid.min_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/z/min_out", -10);
  z_pid.max_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/z/max_out", 10);
  z_pid.controller_name_ = "z";

  roll_pid.p_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/roll/kp", 10);
  roll_pid.i_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/roll/ki", 1);
  roll_pid.d_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/roll/kd", 1);
  roll_pid.min_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/roll/min_out", -10);
  roll_pid.max_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/roll/max_out", 10);
  roll_pid.controller_name_ = "roll";

  pitch_pid.p_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/pitch/kp", 10);
  pitch_pid.i_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/pitch/ki", 1);
  pitch_pid.d_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/pitch/kd", 1);
  pitch_pid.min_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/pitch/min_out", -10);
  pitch_pid.max_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/pitch/max_out", 10);
  pitch_pid.controller_name_ = "pitch";

  yaw_pid.p_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/yaw/kp", 10);
  yaw_pid.i_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/yaw/ki", 1);
  yaw_pid.d_gain_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/yaw/kd", 1);
  yaw_pid.min_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/yaw/min_out", -10);
  yaw_pid.max_out_ = FarolGimmicks::getParameters<double>(nh_private_, "pid/yaw/max_out", 10);
  yaw_pid.controller_name_ = "yaw";

  PID::configure();
}


void PID::configure() {
  // set up the parameters value
  // reset the controller
}

bool PID::compute_force(double Dt) {
  force_[0] = x_pid.compute(position_[0],linear_velocity_[0], position_ref_[0], Dt, false);
  force_[1] = y_pid.compute(position_[1],linear_velocity_[1], position_ref_[1], Dt, false);
  force_[2] = z_pid.compute(position_[2],linear_velocity_[2], position_ref_[2], Dt, false);
  return true;
}


bool PID::compute_torque(double Dt) {
  torque_[0] = roll_pid.compute(attitude_[0],angular_velocity_[0], attitude_ref_[0], Dt, true);
  torque_[1] = pitch_pid.compute(attitude_[1],angular_velocity_[1], attitude_ref_[1], Dt, true);
  torque_[2] = yaw_pid.compute(attitude_[2],angular_velocity_[2], attitude_ref_[2], Dt, true);
  return true;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
//          Attitude PID that i implemented in INDIA and tested in bluerov, here repurposed
/////////////////////////////////////////////////////////////////////////////////////////////////////

PositionPID::PositionPID() = default;

float PositionPID::compute(float state, float state_rate, float state_ref, float Dt, bool angular) {
  
  // convert degrees to radians
  if(angular){
    state = state / 180*M_PI;
    state_rate = state_rate / 180*M_PI;
    state_ref = state_ref / 180*M_PI;
  }
  

  // Compute control input
  float error = state_ref- state;
  if(angular)
    error = wrapToPi(error);

  
  // if first iteration dont compute derivative
  double state_rate_dot=0, state_dot=0, state_rate_dot_filter=0;
  if (first_it_) {
    state_rate_dot = 0;
    state_dot = 0;
  } else {
    state_rate_dot = (state_rate - state_rate_prev_) / Dt;
    // aply a low pass filter because previous computation amplifies noise
    double a = 31.4;                      // pole of the low pass filter
    double lpf_A = std::exp(-a * Dt);   // descretization of the filter
    double lpf_B = 1 - lpf_A;               // descretization of the filter
    state_rate_dot_filter = lpf_A*state_rate_dot_filter_prev_ + state_rate_dot*lpf_B;

    //state_dot = wrapToPi(state - state_prev)/Dt;
    state_dot = state_rate;
  }

  // adding up all PID terms
  double tau_d;
  tau_d =  i_gain_ * error - p_gain_ * state_dot - d_gain_*state_rate_dot_filter;


  // integration with anti windup
  double K_a = 1/Dt;
  double u_dot = tau_d - K_a * (u_prev_ - u_sat_prev_);
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



  // Update prev values
  state_rate_prev_ = state_rate;
  state_prev_ = state;
  state_rate_dot_filter_prev_ = state_rate_dot_filter;

  u_prev_ = u;
  u_sat_prev_ = u_sat;
  ref_prev_ = state_ref;
  first_it_=false;

  return u_sat;
}







