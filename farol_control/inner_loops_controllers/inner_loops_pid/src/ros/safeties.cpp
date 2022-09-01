#include "safeties.h"

Safeties::Safeties(ros::NodeHandle &nh) {
  loadParams(nh);
  initializeSubscribers(nh);
  initializePublishers(nh);
}

Safeties::~Safeties() {}

void Safeties::loadParams(ros::NodeHandle &nh) {
  min_altitude_ = FarolGimmicks::getParameters<double>(nh, "min_alt", 2.0);
}

void Safeties::initializeSubscribers(ros::NodeHandle &nh) {
  depth_sub_ = nh.subscribe(FarolGimmicks::getParameters<std::string>(
    nh, "topics/subscribers/depth", "/ref/depth"),
    10, &Safeties::depthSafetyCallback, this);

  altitude_sub_ = nh.subscribe(FarolGimmicks::getParameters<std::string>(
    nh, "topics/subscribers/altitude", "/ref/altitude"),
    10, &Safeties::altitudeSafetyCallback, this);

  state_sub_ = nh.subscribe(FarolGimmicks::getParameters<std::string>(
    nh, "topics/subscribers/state", "/nav/filter/state"),
    10, &Safeties::stateCallback, this);
}

void Safeties::initializePublishers(ros::NodeHandle &nh) {
  depth_safety_pub_ = nh.advertise<std_msgs::Float64>(
      FarolGimmicks::getParameters<std::string>(
          nh, "topics/subscribers/depth_safety", "/ref/depth_safety"), 1);
  altitude_safety_pub_ = nh.advertise<std_msgs::Float64>(
      FarolGimmicks::getParameters<std::string>(
          nh, "topics/subscribers/altitude_safety", "/ref/altitude_safety"), 1);
}

void Safeties::depthSafetyCallback(const std_msgs::Float64 &msg) {
  double ref_depth = msg.data;
  std_msgs::Float64 out;
  if (state_depth_ + state_altitude_ - ref_depth < min_altitude_) {
    out.data = min_altitude_;
    altitude_safety_pub_.publish(out);
  } else {
    out.data = ref_depth;
    depth_safety_pub_.publish(out);
  }
}

void Safeties::altitudeSafetyCallback(const std_msgs::Float64 &msg) {
  double ref_altitude = msg.data;
  std_msgs::Float64 out;

  ref_altitude = (ref_altitude > min_altitude_) ? ref_altitude : min_altitude_;
  out.data = ref_altitude;
  altitude_safety_pub_.publish(out);
}

void Safeties::stateCallback(const auv_msgs::NavigationStatus &msg) {
  state_depth_ = msg.position.depth;
  state_altitude_ = msg.altitude;
}
