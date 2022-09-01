#include "innerloops.h"
#include "ros/init.h"

Innerloops::Innerloops(ros::NodeHandle &nh) : nh_(nh) {
  
  // Initialize the forces bypass flag
  forces_hard_bypass_ = nh.param("forces_hard_bypass", false);

  // Initialize the timeout (references will be ignored if the last reference received is this time old) [s]
  timeout_ref_ = nh.param("timout_ref", 0.5);
  
  // Initialize all the other ROS nodes and services
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();
}

Innerloops::~Innerloops() { ros::shutdown(); }

void Innerloops::initializeSubscribers() {
  // Angular controllers
  // Yaw
  controllers_.push_back(
      new RosController(nh_, "yaw", 
        FarolGimmicks::getParameters<std::string>(
          nh_, "topics/subscribers/yaw", "yaw_ref"),
          &yaw_, &torque_request_[2], Innerloops::nodeFrequency()));

  controllers_.back()->setCircularUnits(true);

  // Pitch
  controllers_.push_back(
      new RosController(nh_, "pitch",
        FarolGimmicks::getParameters<std::string>(
          nh_, "topics/subscribers/pitch", "pitch_ref"),
          &pitch_, &torque_request_[1], Innerloops::nodeFrequency()));

  controllers_.back()->setCircularUnits(true);

  // Roll
  controllers_.push_back(
      new RosController(nh_, "roll",
        FarolGimmicks::getParameters<std::string>(
          nh_, "topics/subscribers/roll", "roll_ref"),
          &roll_, &torque_request_[0], Innerloops::nodeFrequency()));

  controllers_.back()->setCircularUnits(true);

  // Angular rate controllers
  // Yaw rate
  controllers_.push_back(
      new RosController(nh_, "yaw_rate",
          FarolGimmicks::getParameters<std::string>(
            nh_, "topics/subscribers/yaw_rate", "yaw_rate_ref"),
            &yaw_rate_, &torque_request_[2], Innerloops::nodeFrequency()));

  // Pitch rate
  controllers_.push_back(
      new RosController(nh_, "pitch_rate",
        FarolGimmicks::getParameters<std::string>(
          nh_, "topics/subscribers/pitch_rate", "pitch_rate_ref"),
          &pitch_rate_, &torque_request_[1], Innerloops::nodeFrequency()));

  // Roll rate
  controllers_.push_back(
    new RosController(nh_, "roll_rate",
      FarolGimmicks::getParameters<std::string>(
        nh_, "topics/subscribers/roll_rate", "roll_rate_ref"),
        &roll_rate_, &torque_request_[0], Innerloops::nodeFrequency()));

  // Speed controllers
  // Surge
  controllers_.push_back(
      new RosController(nh_, "surge",
        FarolGimmicks::getParameters<std::string>(
          nh_, "topics/subscribers/surge", "surge_ref"),
          &surge_, &force_request_[0], Innerloops::nodeFrequency()));

  // Sway
  controllers_.push_back(
      new RosController(nh_, "sway",
        FarolGimmicks::getParameters<std::string>(
          nh_, "topics/subscribers/sway", "sway_ref"),
          &sway_, &force_request_[1], Innerloops::nodeFrequency()));

  // Heave
  controllers_.push_back(
      new RosController(nh_, "heave",
        FarolGimmicks::getParameters<std::string>(
          nh_, "topics/subscribers/heave", "heave_ref"),
          &heave_, &force_request_[2], Innerloops::nodeFrequency()));

  // Depth & Altitude controllers
  // Depth
  controllers_.push_back(
      new RosController(nh_, "depth",
        FarolGimmicks::getParameters<std::string>(
          nh_, "topics/subscribers/depth_safety", "depth_ref"),
          &depth_, &force_request_[2], Innerloops::nodeFrequency()));

  // Altitude
  controllers_.push_back(
    new RosController(nh_, "altitude",
      FarolGimmicks::getParameters<std::string>(
        nh_, "topics/subscribers/altitude_safety", "altitude_ref"),
        &altitude_, &force_request_[2], Innerloops::nodeFrequency()));
  controllers_.back()->setPositiveOutput(false);

  // state subscription
  st_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(
              nh_, "topics/subscribers/state", "/nav/filter/state"),
              10, &Innerloops::StateCallback, this);

  // state subscription
  force_bypass_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(
                        nh_, "topics/subscribers/force_bypass", "/force_bypass"),
                        10, &Innerloops::forceBypassCallback, this);
}

void Innerloops::initializeServices() {
  change_ff_gains_srv_ = nh_.advertiseService("/inner_forces/change_ff_gains",
                        &Innerloops::changeFFGainsService, this);
  change_gains_srv_ = nh_.advertiseService("/inner_forces/change_inner_gains",
                        &Innerloops::changeGainsService, this);
  change_limits_srv_ = nh_.advertiseService("/inner_forces/change_inner_limits",
                        &Innerloops::changeLimitsService, this);
}

void Innerloops::initializePublishers() {
  // output forces and torques
  ft_pub_ = nh_.advertise<auv_msgs::BodyForceRequest>(
      FarolGimmicks::getParameters<std::string>(
          nh_, "topics/publishers/thrust_body_request", "/thrust_body_request"), 1);
}

void Innerloops::initializeTimer() {
  // Start Timer
  timer_ = nh_.createTimer(ros::Duration(1.0 / Innerloops::nodeFrequency()),
                           &Innerloops::timerCallback, this);
}

double Innerloops::nodeFrequency() {
  double node_frequency;
  node_frequency = FarolGimmicks::getParameters<double>(nh_, "node_frequency", 5);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

void Innerloops::timerCallback(const ros::TimerEvent &event) {
  // Set force and torque request to zero
  std::memset(force_request_, 0, sizeof force_request_);
  std::memset(torque_request_, 0, sizeof torque_request_);

  // call the pid controllers
  for (std::vector<RosController *>::iterator it = controllers_.begin(); it != controllers_.end(); ++it) {
    (*it)->computeCommand();
  }

  // Publish forces and torques
  auv_msgs::BodyForceRequest output_msg;
  output_msg.header.stamp = ros::Time::now();

  output_msg.wrench.force.x = force_request_[0];
  output_msg.wrench.force.y = force_request_[1];
  output_msg.wrench.force.z = force_request_[2];

  output_msg.wrench.torque.x = torque_request_[0];
  output_msg.wrench.torque.y = torque_request_[1];
  output_msg.wrench.torque.z = torque_request_[2];
  
  // Make sure that the last manual force reference is not too hold  
  if (ros::Time::now() - ref_force_bypass_ < ros::Duration(timeout_ref_)) {

    if(forces_hard_bypass_ == false) {
      // If soft bypass - sum the forces
      // This is usefull if we want to use "for example" the surge inner-loop
      // but manually assign an external force to control the torque about the z-axis
      // i.e. an external yaw controller that is not a PID
      output_msg.wrench.force.x += force_bypass_.wrench.force.x;
      output_msg.wrench.force.y += force_bypass_.wrench.force.y;
      output_msg.wrench.force.z += force_bypass_.wrench.force.z;
      output_msg.wrench.torque.x += force_bypass_.wrench.torque.x;
      output_msg.wrench.torque.y += force_bypass_.wrench.torque.y;
      output_msg.wrench.torque.z += force_bypass_.wrench.torque.z;
    } else {

      // If hard bypass - ignore completely the inner-loops
      output_msg.wrench.force.x = force_bypass_.wrench.force.x;
      output_msg.wrench.force.y = force_bypass_.wrench.force.y;
      output_msg.wrench.force.z = force_bypass_.wrench.force.z;
      output_msg.wrench.torque.x = force_bypass_.wrench.torque.x;
      output_msg.wrench.torque.y = force_bypass_.wrench.torque.y;
      output_msg.wrench.torque.z = force_bypass_.wrench.torque.z;
    }
    
  }

  ft_pub_.publish(output_msg);
}

void Innerloops::forceBypassCallback(const auv_msgs::BodyForceRequest &msg) {
  ref_force_bypass_ = ros::Time::now();
  force_bypass_.wrench.force.x = msg.wrench.force.x;
  force_bypass_.wrench.force.y = msg.wrench.force.y;
  force_bypass_.wrench.force.z = msg.wrench.force.z;
  force_bypass_.wrench.torque.x = msg.wrench.torque.x;
  force_bypass_.wrench.torque.y = msg.wrench.torque.y;
  force_bypass_.wrench.torque.z = msg.wrench.torque.z;
}

void Innerloops::StateCallback(const auv_msgs::NavigationStatus &msg) {
  // Save the state into variables to be used separately

  // controller state variables
  roll_ = msg.orientation.x;
  pitch_ = msg.orientation.y;
  yaw_ = msg.orientation.z;
  roll_rate_ = msg.orientation_rate.x;
  pitch_rate_ = msg.orientation_rate.y;
  yaw_rate_ = msg.orientation_rate.z;

  depth_ = msg.position.depth;
  altitude_ = msg.altitude;

  surge_ = msg.body_velocity.x;
  sway_ = msg.body_velocity.y;
  heave_ = msg.body_velocity.z;

  vdepth_ = msg.seafloor_velocity.z;
  valtitude_ = -msg.seafloor_velocity.z;
}

bool Innerloops::changeFFGainsService(
    inner_loops_pid::ChangeFFGains::Request &req,
    inner_loops_pid::ChangeFFGains::Response &res) {

  bool control_changed{false};

  for (auto &controller : controllers_) {
    if ((controller->getControllerName().size() == req.inner_type.size()) &&
        std::equal(req.inner_type.begin(), req.inner_type.end(),
                   controller->getControllerName().begin(),
                   [](char &c1, char &c2) {
                     return (c1 == c2 || std::toupper(c1) == std::toupper(c2));
                   })) {
      controller->setFFGainsPID(req.kff, req.kff_d, req.kff_lin_drag, req.kff_quad_drag);
      control_changed = true;
      break;
    }
  }

  if (!control_changed) {
    res.success = false;
    res.message += "Bad control name " + req.inner_type;
  } else {
    res.success = true;
    res.message += "New " + req.inner_type + " feedfoward gains are" +
                   " Squared Proportional FF: " + std::to_string(req.kff) +
                   " Derivative Proportional FF: " + std::to_string(req.kff_d) +
                   " Linear Drag FF: " + std::to_string(req.kff_lin_drag) +
                   " Quadratic Drag FF: " + std::to_string(req.kff_quad_drag);
  }

  return true;
}

bool Innerloops::changeGainsService(
    inner_loops_pid::ChangeInnerGains::Request &req,
    inner_loops_pid::ChangeInnerGains::Response &res) {

  bool control_changed{false};

  for (auto &controller : controllers_) {
    if ((controller->getControllerName().size() == req.inner_type.size()) &&
        std::equal(req.inner_type.begin(), req.inner_type.end(),
                   controller->getControllerName().begin(),
                   [](char &c1, char &c2) {
                     return (c1 == c2 || std::toupper(c1) == std::toupper(c2));
                   })) {
      controller->setGainsPID(req.kp, req.ki, req.kd);
      control_changed = true;
      break;
    }
  }

  if (!control_changed) {
    res.success = false;
    res.message += "Bad control name " + req.inner_type;
  } else {
    res.success = true;
    res.message += "New " + req.inner_type + " gains are" +
                   " kp: " + std::to_string(req.kp) +
                   " ki: " + std::to_string(req.ki) +
                   " kd: " + std::to_string(req.kd);
  }

  return true;
}

bool Innerloops::changeLimitsService(
    inner_loops_pid::ChangeInnerLimits::Request &req,
    inner_loops_pid::ChangeInnerLimits::Response &res) {

  bool control_changed{false};

  for (auto &controller : controllers_) {
    if ((controller->getControllerName().size() == req.inner_type.size()) &&
        std::equal(req.inner_type.begin(), req.inner_type.end(),
                   controller->getControllerName().begin(),
                   [](char &c1, char &c2) {
                     return (c1 == c2 || std::toupper(c1) == std::toupper(c2));
                   })) {
      controller->setLimitBoundsPID(req.max_out, req.min_out);
      control_changed = true;
      break;
    }
  }

  if (!control_changed) {
    res.success = false;
    res.message += "Bad control name " + req.inner_type;
  } else {
    res.success = true;
    res.message += "New " + req.inner_type + " limits are" +
                   " max_out: " + std::to_string(req.max_out) +
                   " min_out: " + std::to_string(req.min_out);
  }

  return true;
}
