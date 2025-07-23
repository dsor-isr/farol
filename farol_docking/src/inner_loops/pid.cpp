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

  debug_pub = nh_private_.advertise<farol_docking::PID_Debug>(FarolGimmicks::getParameters<std::string>(nh_private_, "PID/publishers/debug", "medusa_amarelo_zero/docking/pid/debug"), 5);
  change_gains_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, "PID/services/change_inner_gains", "medusa_amarelo_zero/docking/PID/inner_forces/change_inner_gains"), &PID::changeGainsService, this);
  sub_reset_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "PID/subscribers/reset", "medusa_amarelo_zero/docking/inner_loops/pid/reset"), 1, &PID::reset_callback, this);

}

void PID::configure() {/* Done in the constructor */}

void PID::reset_xyz(){
  x_pid.u_prev_ = 0;
  y_pid.u_prev_ = 0;
  z_pid.u_prev_ = 0;
}


void PID::reset_rpy(){
  yaw_pid.u_prev_ = 0;
  pitch_pid.u_prev_ = 0;
  roll_pid.u_prev_ = 0;
}



bool PID::compute_force(double Dt) {
  Eigen::Matrix3d R = (Eigen::AngleAxisd(-attitude_(2)/180*M_PI, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  Eigen::Vector3d body_pos = R.transpose()* position_;
  Eigen::Vector3d body_ref = R.transpose() * position_ref_;

  force_[0] = x_pid.compute(body_pos[0],linear_velocity_[0], body_ref[0], Dt, false);
  force_[1] = y_pid.compute(body_pos[1],linear_velocity_[1], body_ref[1], Dt, false);
  force_[2] = z_pid.compute(body_pos[2],linear_velocity_[2], body_ref[2], Dt, false);
  return true;
}


bool PID::compute_torque(double Dt) {
  torque_[0] = roll_pid.compute(attitude_[0],angular_velocity_[0], attitude_ref_[0], Dt, true);
  torque_[1] = pitch_pid.compute(attitude_[1],angular_velocity_[1], attitude_ref_[1], Dt, true);
  torque_[2] = yaw_pid.compute(attitude_[2],angular_velocity_[2], attitude_ref_[2], Dt, true);
  return true;
}

void PID::reset_callback(const std_msgs::Empty &msg) {
  PID::reset_xyz();
  PID::reset_rpy();
}



bool PID::changeGainsService( inner_loops_pid::ChangeInnerGains::Request &req, inner_loops_pid::ChangeInnerGains::Response &res) {
  
  bool control_changed = false;

  if(req.inner_type == "yaw"){
    yaw_pid.p_gain_ = req.kp;
    yaw_pid.i_gain_ = req.ki;
    yaw_pid.d_gain_ = req.kd;
    control_changed = true;
    PID::reset_rpy();
  }else if(req.inner_type == "x"){
    x_pid.p_gain_ = req.kp;
    x_pid.i_gain_ = req.ki;
    x_pid.d_gain_ = req.kd;
    control_changed = true;
    PID::reset_xyz();
  }else if(req.inner_type == "y"){
    y_pid.p_gain_ = req.kp;
    y_pid.i_gain_ = req.ki;
    y_pid.d_gain_ = req.kd;
    control_changed = true;
    PID::reset_xyz();
  }else if(req.inner_type == "z"){
    z_pid.p_gain_ = req.kp;
    z_pid.i_gain_ = req.ki;
    z_pid.d_gain_ = req.kd;
    control_changed = true;
    PID::reset_xyz();
  }

  if (!control_changed) {
    res.success = false;
    res.message += "Bad control name " + req.inner_type;
  } else {
    res.success = true;
    res.message += "[Integrators Reset] + New " + req.inner_type + " gains are" +
                   " kp: " + std::to_string(req.kp) +
                   " ki: " + std::to_string(req.ki) +
                   " kd: " + std::to_string(req.kd);
  }
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
  
  ROS_INFO_STREAM("P: " << p_gain_ <<", D: " << d_gain_ << ", I: " << i_gain_);
  // Compute control input
  float error = state_ref- state; 
  // ROS_INFO_STREAM(controller_name_ << "::  error: " << error <<" state_ref: "<< state_ref <<" - "<<state <<" and rate: " <<state_rate);
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
  first_it_=false;

  return u_sat;
}







