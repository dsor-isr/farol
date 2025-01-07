/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/

#include "waypoint.h"

WaypointNode::WaypointNode(ros::NodeHandle *nodehandle,
                               ros::NodeHandle *nodehandle_private)
    : nh_(*nodehandle), nh_p_(*nodehandle_private) {
  ROS_INFO("in class constructor of WaypointNode");
  loadParams();
  initializeSubscribers();
  initializeServices();
  initializePublishers();
  initializeTimer();
}

WaypointNode::~WaypointNode() {

  // Shutdown subsctribers
  flag_sub_.shutdown();
  state_sub_.shutdown();

  // Shutdown publishers
  u_ref_pub_.shutdown();
  yaw_ref_pub_.shutdown();
  flag_pub_.shutdown();

  // stop timer
  timer_.stop();

  // shutdown node
  nh_.shutdown();
  nh_p_.shutdown();
}

void WaypointNode::loadParams() {
  ROS_INFO("Load the WaypointNode parameters");
  cdist_ = FarolGimmicks::getParameters<double>(nh_p_, "cdist");
  delta_t_ = FarolGimmicks::getParameters<double>(nh_p_, "delta_t");

  // waypoint type 1 gains
  ku_ = FarolGimmicks::getParameters<double>(nh_p_, "type1/gains/ku");
  ks_ = FarolGimmicks::getParameters<double>(nh_p_, "type1/gains/ks");
  speed_turn_ = FarolGimmicks::getParameters<double>(
      nh_p_, "type1/gains/speed_turn", 0.0); // used in loose controller

  // waypoint type 2 (with heading) gains
  k1_ = FarolGimmicks::getParameters<double>(nh_p_, "type2/gains/k1");
  k2_ = FarolGimmicks::getParameters<double>(nh_p_, "type2/gains/k2");
  k3_ = FarolGimmicks::getParameters<double>(nh_p_, "type2/gains/k3");

  // Topics
  yaw_ref_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/publishers/yaw_ref", "YawRef");
  u_ref_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/publishers/u_ref", "URef");
  v_ref_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/publishers/v_ref", "/ref/sway");
  yaw_rate_ref_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/publishers/yaw_rate_ref", "/ref/yaw_rate");
  flag_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/publishers/flag", "Flag");
  state_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/subscribers/state", "State");
  turn_radius_flag_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/subscribers/turn_radius_flag", "turn_radius_flag");
  wp_standard_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/services/wp_standard", "/controls/send_wp_standard");
  wp_loose_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/services/wp_loose", "/controls/send_wp_loose");
  wp_heading_topic_ = FarolGimmicks::getParameters<std::string>(
      nh_p_, "topics/services/wp_heading", "/controls/send_wp_heading");
}

void WaypointNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for WaypointNode");
  flag_sub_ =
      nh_.subscribe(flag_topic_, 10, &WaypointNode::flagCallback, this);
  state_sub_ =
      nh_.subscribe(state_topic_, 10, &WaypointNode::updateCallback, this);
  turn_radius_flag_sub_ =
      nh_.subscribe(turn_radius_flag_topic_, 10, &WaypointNode::turnRadiusFlagCallback, this);
}

void WaypointNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for WaypointNode");
  yaw_ref_pub_ = nh_.advertise<std_msgs::Float64>(yaw_ref_topic_, 10);
  yaw_rate_ref_pub_ = nh_.advertise<std_msgs::Float64>(yaw_rate_ref_topic_, 10);
  u_ref_pub_ = nh_.advertise<std_msgs::Float64>(u_ref_topic_, 10);
  v_ref_pub_ = nh_.advertise<std_msgs::Float64>(v_ref_topic_, 10);
  flag_pub_ = nh_.advertise<std_msgs::Int8>(flag_topic_, 10);
}

void WaypointNode::initializeServices() {
  ROS_INFO("Initializing Services for WaypointNode");
  wp_standard_srv_ = nh_.advertiseService(
      wp_standard_topic_, &WaypointNode::sendWpStandardService, this);
  wp_loose_srv_ = nh_.advertiseService(
      wp_loose_topic_, &WaypointNode::sendWpLooseService, this);
  wp_heading_srv_ = nh_.advertiseService(
      wp_heading_topic_, &WaypointNode::sendWpHeadingService, this);
}

void WaypointNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / WaypointNode::nodeFrequency()),
                           &WaypointNode::timerCallback, this);
  timer_.stop();
}

double WaypointNode::nodeFrequency() {
  node_frequency_ =
      FarolGimmicks::getParameters<double>(nh_p_, "node_frequency", 10.0);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency_);
  return node_frequency_;
}

void WaypointNode::timerCallback(const ros::TimerEvent &event) {
  // compute waypoint controller and publish
  wp_controller_->compute(veh_state_, wp_ref_, turn_radius_flag_);
}

void WaypointNode::updateCallback(const auv_msgs::NavigationStatus &msg) {
  // update vehicle state
  veh_state_.eta1[0] = msg.position.north;
  veh_state_.eta1[1] = msg.position.east;
  veh_state_.eta2[2] = msg.orientation.z;
  veh_state_.v1[0] = msg.body_velocity.x;
  veh_state_.v1[1] = msg.body_velocity.y;

  // send message if error and stop timer
  if (!(msg.status & msg.STATUS_ALL_OK) && timer_.hasPending()) {
    ROS_ERROR("The filter estimate is not good, disabling WayPoint");
    timer_.stop();
  }
}

void WaypointNode::flagCallback(const std_msgs::Int8 &msg) {
  // stop the waypoint controller if the flag has been changed to other value
  // than 4
  if (timer_.hasStarted() && msg.data != 4) {
    timer_.stop();
    ROS_INFO("Some Process changed the flag to %d", msg.data);
  }
}

void WaypointNode::turnRadiusFlagCallback(const std_msgs::Bool &msg) {
  turn_radius_flag_ = msg.data;
}

void WaypointNode::createWaypoint(WaypointController *new_wp) {
  // free memory from waypoint controller pointer and point it to new controller
  // if (wp_controller_) {
  //   free(wp_controller_);
  // }
  wp_controller_ = new_wp;
}

bool WaypointNode::decodeWaypoint(double x, double y) {
  Eigen::Vector2d ref_return;
  wp_ref_.eta1[0] = x;
  wp_ref_.eta1[1] = y;
  if (x == -1 && y == -1) {               // Hold in the same position
    wp_ref_.eta1[0] = veh_state_.eta1[0]; // Actual GPS position
    wp_ref_.eta1[1] = veh_state_.eta1[1]; // Actual GPS position
  }

  else if (x == -3 && y == -3) {
    wp_ref_.eta1[0] = veh_state_.eta1[0] +
                      veh_state_.v1[0] *
                          cos(veh_state_.eta2[2] * (FarolGimmicks::PI / 180)) *
                          delta_t_;
    wp_ref_.eta1[1] = veh_state_.eta1[1] +
                      veh_state_.v1[0] *
                          sin(veh_state_.eta2[2] * (FarolGimmicks::PI / 180)) *
                          delta_t_;
  }
  // Verify the stop condition
  else if (x == -2 && y == -2) {
    return false;
  }
  return true;
}

bool WaypointNode::sendWpStandardService(
    waypoint::sendWpType1::Request &req,
    waypoint::sendWpType1::Response &res) {

  ROS_INFO("Sending Waypoint");

  // create pointer to new controller
  WaypointController *aux_wp = new WpStandard(u_ref_pub_, yaw_ref_pub_);
  // set the gains
  aux_wp->setGains(std::vector<double>{cdist_, ku_, ks_});
  // substitute node pointer of the controller
  createWaypoint(aux_wp);

  // set the new waypoint reference, change the flag value and start the main
  // loop
  if (!decodeWaypoint(req.x, req.y)) {
    res.message += "Stop signal sent";
    ROS_WARN("Flag set to 0: waypoint node couldn't decode waypoint.");
    FarolGimmicks::publishValue<std_msgs::Int8, const int>(flag_pub_, 0);
    timer_.stop();
  } else {
    res.success = true;
    res.message += "New waypoint reference: (" +
                   std::to_string(wp_ref_.eta1[0]) + "," +
                   std::to_string(wp_ref_.eta1[1]) + ")";
    FarolGimmicks::publishValue<std_msgs::Int8, const int>(flag_pub_, 4);
    if (!timer_.hasStarted()) {
      timer_.start();
    }
  }
  return true;
}

bool WaypointNode::sendWpLooseService(
    waypoint::sendWpType1::Request &req,
    waypoint::sendWpType1::Response &res) {

  ROS_INFO("Sending Waypoint");
  // create pointer to new controller
  WaypointController *aux_wp = new WpLoose(u_ref_pub_, yaw_ref_pub_);
  // set the gains
  aux_wp->setGains(std::vector<double>{cdist_, ku_, ks_, speed_turn_});
  aux_wp->setFrequency(node_frequency_);
  // substitute node pointer of the controller
  createWaypoint(aux_wp);

  // set the new waypoint reference, change the flag value and start the main
  // loop
  if (!decodeWaypoint(req.x, req.y)) {
    res.message += "Stop signal sent";
    FarolGimmicks::publishValue<std_msgs::Int8, const int>(flag_pub_, 0);
    timer_.stop();
  } else {
    res.success = true;
    res.message += "New waypoint reference: (" +
                   std::to_string(wp_ref_.eta1[0]) + "," +
                   std::to_string(wp_ref_.eta1[1]) + ")";
    FarolGimmicks::publishValue<std_msgs::Int8, const int>(flag_pub_, 4);
    if (!timer_.hasStarted()) {
      timer_.start();
    }
  }
  return true;
}

bool WaypointNode::sendWpHeadingService(
    waypoint::sendWpType1::Request &req,
    waypoint::sendWpType1::Response &res) {

  ROS_INFO("Sending Waypoint");
  // create pointer to new controller
  WaypointController *aux_wp =
      new WpHeading(u_ref_pub_, v_ref_pub_, yaw_rate_ref_pub_);
  // set the gains
  aux_wp->setGains(std::vector<double>{cdist_, k1_, k2_, k3_});
  aux_wp->setFrequency(node_frequency_);
  // substitute node pointer of the controller
  createWaypoint(aux_wp);
  // set the new waypoint reference, change the flag value and start the main
  // loop
  if (!decodeWaypoint(req.x, req.y)) {
    res.message += "Stop signal sent";
    FarolGimmicks::publishValue<std_msgs::Int8, const int>(flag_pub_, 0);
    timer_.stop();
  } else {
    // add the yaw reference here since the wp logic (decodeWaypoint) doesnt
    // include it
    wp_ref_.eta2[2] = req.yaw;
    res.success = true;
    res.message += "New waypoint reference: (" +
                   std::to_string(wp_ref_.eta1[0]) + "," +
                   std::to_string(wp_ref_.eta1[1]) + "," + std::to_string(wp_ref_.eta2[2]) + ")";
    FarolGimmicks::publishValue<std_msgs::Int8, const int>(flag_pub_, 4);
    if (!timer_.hasStarted()) {
      timer_.start();
    }
  }
  return true;
}
