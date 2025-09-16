// Implementation of the inner loop for docking
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Handles core filtering logic for docking using exponential smoothing
// #include <farol_docking/inner_loops/trajectory_tracking_node.hpp>  



#include <ros/ros.h>
#include <Eigen/Dense>
#include <farol_docking/trajectory_tracking/trajectory_tracking_node.hpp>
#include <farol_docking/trajectory_tracking/se3_tracker.hpp>

static inline double deg2rad(double d){ return d*M_PI/180.0; }


TrajectoryTrackingNode::TrajectoryTrackingNode(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
: nh_(*nodehandle), nh_private_(*nodehandle_private)
{

  // Parameters
  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "debug", false);
  controller_type_ = FarolGimmicks::getParameters<std::string>(nh_private_, "type", "se3_tracker");
  reference_frame_ = FarolGimmicks::getParameters<std::string>(nh_private_, "reference_frame", "dock");

  // Subscribers
  sub_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/docking_state", "docking/state"), 1, &TrajectoryTrackingNode::state_callback, this);
  sub_se3_ref_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/trajectory", "docking/trajectory"), 1, &TrajectoryTrackingNode::se3_ref_callback, this);
  // Publishers
  force_request_pub_ = nh_private_.advertise<auv_msgs::BodyForceRequest>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/force", "/force_bypass"), 5);

  // Timer
  timer_ = nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &TrajectoryTrackingNode::timerIterCallback, this);

  // Controller
  if(controller_type_ == "se3_tracker") {
    controller_ = std::make_unique<Se3Tracker>(&nh_, &nh_private_);
    } else {
    ROS_FATAL_STREAM("Unknown controller type: " << controller_type_);
    throw std::runtime_error("Unknown controller type");
  }
}

TrajectoryTrackingNode::~TrajectoryTrackingNode(){
  force_request_pub_.shutdown();
}



void TrajectoryTrackingNode::state_callback(const auv_msgs::NavigationStatus &msg)
{
  // Position in selected frame (adjust field names to your message)
  controller_->position_ << msg.local_position.x, msg.local_position.y, msg.local_position.z;


  // Build attitude matrix R_ from local_attitude (assumed degrees here; change if radians)
  const double yaw = deg2rad(msg.local_attitude.yaw);
  const double pitch = deg2rad(msg.local_attitude.pitch);
  const double roll = deg2rad(msg.local_attitude.roll);
  const Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
  controller_->R_ = (Rz*Ry*Rx).toRotationMatrix();


  // Body velocities
  controller_->v_ << msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z;
  controller_->w_ << deg2rad(msg.orientation_rate.x), deg2rad(msg.orientation_rate.y), deg2rad(msg.orientation_rate.z);
}


void TrajectoryTrackingNode::se3_ref_callback(const farol_docking::SE3Ref &msg)
{
  controller_->p_d_ = {msg.p.x, msg.p.y, msg.p.z};
  controller_->pd_d_ = {msg.pd.x, msg.pd.y, msg.pd.z};
  controller_->pdd_d_ = {msg.pdd.x,msg.pdd.y,msg.pdd.z};


  Eigen::Quaterniond qd(msg.q.w, msg.q.x, msg.q.y, msg.q.z);
  qd.normalize(); controller_->R_d_ = qd.toRotationMatrix();


  controller_->w_d_ = {msg.wd.x, msg.wd.y, msg.wd.z};
  controller_->wdd_d_ = {msg.wdd.x, msg.wdd.y, msg.wdd.z};


  t_ref_ = ros::Time::now().toSec(); //msg.header.stamp.toSec();
  // Optional axis disables
  if(msg.disable_axis.size() == 6){
    for(int i=0;i<6;++i) disable_axis_[i] = msg.disable_axis[i];
  }
}


void TrajectoryTrackingNode::timerIterCallback(const ros::TimerEvent &)
{
  const double now = ros::Time::now().toSec();
  const double dt = first_it_ ? 0.0 : (now - last_it_time_);
  last_it_time_ = now;
  if(first_it_) { first_it_ = false; return; }


  // Require a recent reference
  if (now - t_ref_ > 0.2) return;


  controller_->compute_wrench(dt);


  // Optional runtime axis masking
  if(disable_axis_[0]) controller_->force_.x() = 0.0;
  if(disable_axis_[1]) controller_->force_.y() = 0.0;
  if(disable_axis_[2]) controller_->force_.z() = 0.0;
  if(disable_axis_[3]) controller_->torque_.x() = 0.0;
  if(disable_axis_[4]) controller_->torque_.y() = 0.0;
  if(disable_axis_[5]) controller_->torque_.z() = 0.0;


  auv_msgs::BodyForceRequest out;
  out.header.stamp = ros::Time::now();
  out.header.frame_id = "base_link"; // set appropriately
  out.wrench.force.x = controller_->force_.x();
  out.wrench.force.y = controller_->force_.y();
  out.wrench.force.z = controller_->force_.z();
  out.wrench.torque.x = controller_->torque_.x();
  out.wrench.torque.y = controller_->torque_.y();
  out.wrench.torque.z = controller_->torque_.z();
  force_request_pub_.publish(out);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_tracking");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TrajectoryTrackingNode node(&nh, &nh_private);
  ros::spin();
  return 0;
}


