#include "ros_controller.h"

// General Controller
RosController::RosController(ros::NodeHandle &nh, std::string controller_name,
                             std::string refCallback_topic, double *state,
                             double *force_or_torque, double frequency)
    : state_ptr_(state), controller_name_(controller_name),
      force_or_torque_ptr_(force_or_torque),  frequency_(frequency) {
  init(nh, controller_name, refCallback_topic);
}

// Attitude Controller
RosController::RosController(ros::NodeHandle &nh, std::string controller_name,
                             std::string refCallback_topic, double *state, double *attitude_rate,
                             double *force_or_torque, double frequency,
                             bool *turn_limiter_flag, double *turn_radius_speed, RateLimiter *rate_limiter, double *turn_radius_speed_t)
: state_ptr_(state), attitude_rate_ptr_(attitude_rate), controller_name_(controller_name),
force_or_torque_ptr_(force_or_torque),  frequency_(frequency),
turn_limiter_flag_ptr_(turn_limiter_flag), turn_radius_speed_(turn_radius_speed), rate_limiter_ptr_(rate_limiter), turn_radius_speed_t_(turn_radius_speed_t) {
init(nh, controller_name, refCallback_topic);
}

// Yaw_rate controller
RosController::RosController(ros::NodeHandle &nh, std::string controller_name,
                             std::string refCallback_topic, double *state,
                             double *force_or_torque, double frequency,
                             bool *turn_limiter_flag, double *turn_radius_speed, double *turn_radius_speed_t)
    : state_ptr_(state), controller_name_(controller_name),
      force_or_torque_ptr_(force_or_torque),  frequency_(frequency),
      turn_limiter_flag_ptr_(turn_limiter_flag), turn_radius_speed_(turn_radius_speed), turn_radius_speed_t_(turn_radius_speed_t) {
  init(nh, controller_name, refCallback_topic);
}

// Altitude controller
RosController::RosController(ros::NodeHandle &nh, std::string controller_name,
                             std::string refCallback_topic, double *state, double *altitude_rate,
                             double *force_or_torque, double frequency)
    : state_ptr_(state), altitude_rate_ptr_(altitude_rate), controller_name_(controller_name),
      force_or_torque_ptr_(force_or_torque),  frequency_(frequency) {
  init(nh, controller_name, refCallback_topic);
}

void RosController::init(ros::NodeHandle &nh, std::string controller_name,
                         std::string refCallback_topic) {
  // default state not angle units
  setCircularUnits(false);
  // positive output sign
  setPositiveOutput(true);

  // read parameters
  double timout = nh.param("timout_ref", 1.5);
  double kp = nh.param("controllers/" + controller_name + "/kp", 0.0);
  double ki = nh.param("controllers/" + controller_name + "/ki", 0.0);
  double kd = nh.param("controllers/" + controller_name + "/kd", 0.0);
  double kff = nh.param("controllers/" + controller_name + "/kff", 0.0);
  double kff_d = nh.param("controllers/" + controller_name + "/kff_d", 0.0);
  double kff_lin_drag = nh.param("controllers/" + controller_name + "/kff_lin_drag", 0.0);
  double kff_quad_drag = nh.param("controllers/" + controller_name + "/kff_quad_drag", 0.0);
  double max_out = nh.param("controllers/" + controller_name + "/max_out", 0.0);
  double min_out = nh.param("controllers/" + controller_name + "/min_out", -max_out);
  double max_error = nh.param("controllers/" + controller_name + "/max_err", 0.0);
  double min_error = nh.param("controllers/" + controller_name + "/min_err", -max_error);
  max_ref_value_ = nh.param("controllers/" + controller_name + "/max_ref", 0.0);
  min_ref_value_ = nh.param("controllers/" + controller_name + "/min_ref", 0.0);
  debug_ = nh.param("controllers/" + controller_name + "/debug", false);
  bool enabled = nh.param("controllers/" + controller_name + "/enabled", false);

  min_turn_radius_ = nh.param("min_turn_radius", 1.0);
  turn_radius_speed_t_max_ = nh.param("turn_radius_t_max", 5.0);

  // Don't create the controller if no gains were specified
  if ((kp == 0.0 && ki == 0.0 && kd == 0.0 && kff == 0.0 && kff_d == 0.0 && kff_lin_drag == 0.0 && kff_quad_drag == 0.0) || enabled == false) {
    ROS_WARN_STREAM("No PID and Feedfoward gains were specified for " + controller_name + " controller.");
    pid_c_ = NULL;
    return;
  }

  // If we are debugging, create a publisher for debug data related to this controller
  if (debug_) {
    ROS_WARN_STREAM("Debugging extra info for " + controller_name + " controller.");

    debug_pub_ = nh.advertise<farol_msgs::mPidDebug>(
      FarolGimmicks::getParameters<std::string>(
          nh, "topics/publishers/debug/" + controller_name, "/debug/" + controller_name), 1);
  }

  // subscribe to relevant topic
  ros_sub_ = nh.subscribe(refCallback_topic.c_str(), 10,
                          &RosController::refCallback, this);

  // create the PID controller with/without low pass filter included
  if ( nh.hasParam("controllers/" + controller_name + "/lpf_fc") ) {
    double lpf_dt, lpf_fc;   
    lpf_dt = 1.0 / frequency_;

    nh.getParam("controllers/" + controller_name + "/lpf_fc", lpf_fc); 

    if (lpf_fc <= 0.0) {
      ROS_WARN_STREAM("Low pass filter cutoff frequency must be higher than 0.");
      pid_c_ = NULL;
      return;
    }
    
    pid_c_ = new PID_Controller(kp, ki, kd, kff, kff_d, kff_lin_drag, kff_quad_drag, max_error, max_out, min_error, min_out, lpf_dt, lpf_fc);
  } 
  else {
    pid_c_ = new PID_Controller(kp, ki, kd, kff, kff_d, kff_lin_drag, kff_quad_drag, max_error, max_out, min_error, min_out);
  }
  
  // initialize variables
  ref_value_ = 0.0;
  ref_time_ = ros::Time(0.0);
  timeout_ref_ = ros::Duration(timout);
  last_cmd_ = ros::Time::now();
}

void RosController::refCallback(const std_msgs::Float64 &msg) {
  ref_time_ = ros::Time::now();
  if (max_ref_value_ == 0.0 && min_ref_value_ == 0)
    ref_value_ = msg.data;
  else
    // saturate references
    ref_value_ = fmin(fmax(msg.data, min_ref_value_), max_ref_value_);



}

double RosController::computeCommand() {

  if (!validRef()) {
    if (debug_) {
      debug_msg_.ref = 0.0;
      debug_msg_.ref_d = 0.0;
      debug_msg_.ref_d_filtered = 0.0;
      debug_msg_.state = *state_ptr_;
      debug_msg_.error = 0.0;
      debug_msg_.error_saturated = 0.0;
      debug_msg_.ffTerm = 0.0;
      debug_msg_.ffDTerm = 0.0;
      debug_msg_.ffDragTerm = 0.0;
      debug_msg_.pTerm = 0.0;
      debug_msg_.iTerm = 0.0;
      debug_msg_.dTerm = 0.0;
      debug_msg_.output = 0.0;
    
      if (circular_units_) {
        if (debug_msg_.state > 180)
          debug_msg_.state -= 360;

        if (debug_msg_.state < -180)
          debug_msg_.state += 360;
      }

      debug_msg_.header.stamp = ros::Time::now();
      debug_msg_.controller = controller_name_;

      debug_pub_.publish(debug_msg_);
    }

    return 0.0;
  }

  // Turning Radius Limiter calculations using RateLimiter object (see dsor_utils)
  if ( (controller_name_ == "yaw" || controller_name_ == "yaw_rate") && *turn_limiter_flag_ptr_) {

    if (min_turn_radius_ <= 0.0) {
      ROS_WARN_STREAM("Minimum turn limit radius must be higher than 0. Ignoring " + controller_name_ + " reference.");
      return 0.0;
    }

    double no_response_turn_radius_speed = ros::Time::now().toSec() - *turn_radius_speed_t_;
    if ( no_response_turn_radius_speed >= turn_radius_speed_t_max_ ) {
      ROS_WARN_STREAM("No measurements received from turn_radius_speed callback for " << no_response_turn_radius_speed << "seconds.\n" + controller_name_ + " controller will not provide output.");
      return 0.0;
    }

    if (controller_name_ == "yaw") {
      
      double rate_limit = (*turn_radius_speed_ / min_turn_radius_) * ( 180 / M_PI );

      rate_limiter_ptr_->setNewRateLimit(rate_limit);

      ref_value_ = DSOR::wrapTo360<double>(rate_limiter_ptr_->Calculate(ref_value_));
    }
    else if (controller_name_ == "yaw_rate") {

      double max_yaw_rate = (*turn_radius_speed_ / min_turn_radius_) * ( 180 / M_PI );

      ref_value_ = DSOR::saturation<double>(ref_value_, -max_yaw_rate, max_yaw_rate);
    }
  }



  double error = ref_value_ - *state_ptr_;
  if (isnan(ref_value_)) {
    ROS_ERROR("getting NaN in %s controller", controller_name_.c_str());
    return 0.0;
  }

  // Wrap the error between [-180,180]
  if (circular_units_) {
    if (error > 180)
      error -= 360;

    if (error < -180)
      error += 360;
  }

  ros::Time tnow = ros::Time::now();  


  // Call the controller
  // if (controller_name_ == "yaw") {
  //   *force_or_torque_ptr_ += pid_c_->computeCommandYaw(*state_ptr_, *attitude_rate_ptr_, ref_value_, (tnow - last_cmd_).toSec(), frequency_);
  // } 
  // Attitude Controllers  
  if (controller_name_ == "pitch" || controller_name_ == "yaw" || controller_name_ == "roll" ) {
    *force_or_torque_ptr_ += pid_c_->computeCommandAttitude(*state_ptr_, *attitude_rate_ptr_, ref_value_, (tnow - last_cmd_).toSec(), debug_);
  } 
  // Speed Controllers
  else if (controller_name_ == "surge" || controller_name_ == "sway" || controller_name_ == "heave") {
    *force_or_torque_ptr_ += pid_c_->computeCommandSpeed(*state_ptr_, ref_value_, (tnow - last_cmd_).toSec(), debug_);
  }
  // TODO: this shoul be for altitude and depth 
  else if (controller_name_ == "altitude") {
    *force_or_torque_ptr_ += pid_c_->computeCommandAltitude(*state_ptr_, *altitude_rate_ptr_, ref_value_, (tnow - last_cmd_).toSec(), frequency_);
  } 
  else {
    *force_or_torque_ptr_ += (positive_output_ ? 1 : -1) * pid_c_->computeCommand(error, ref_value_, (tnow - last_cmd_).toSec(), debug_);
  }
  
  last_cmd_ = tnow;

  // If debugging info, publish the internal controller variables for analysis
  if (debug_) {
    debug_msg_ = pid_c_->getDebugInfo();
    
    debug_msg_.state = *state_ptr_;
    
    if (circular_units_) {
      if (debug_msg_.state > 180)
        debug_msg_.state -= 360;

      if (debug_msg_.state < -180)
        debug_msg_.state += 360;
    }

    debug_msg_.header.stamp = tnow;
    debug_msg_.controller = controller_name_;

    debug_pub_.publish(debug_msg_);
  }

  return *force_or_torque_ptr_;
}

bool RosController::validRef() {
  if (pid_c_ == NULL)
    return false;

  if ((ros::Time::now() - ref_time_) < timeout_ref_) {
    // reactivate the controller if needed
    if (pid_c_->disable) {
      last_cmd_ = ros::Time::now();
      pid_c_->reset();
      pid_c_->disable = false;
    }
    return true;
  }
  pid_c_->disable = true; // disable controller
  return false;
}


