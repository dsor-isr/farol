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


InnerLoopNode::InnerLoopNode(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
: nh_(*nodehandle), nh_private_(*nodehandle_private)
{

  // Parameters
  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "debug", false);
  controller_type_ = FarolGimmicks::getParameters<std::string>(nh_private_, "type", "se3_tracker");
  reference_frame_ = FarolGimmicks::getParameters<std::string>(nh_private_, "reference_frame", "dock");

  // Subscribers
  sub_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/docking_state", "docking/state"), 1, &InnerLoopNode::state_callback, this);
  sub_se3_ref_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/trajectory", "docking/trajectory"), 1, &InnerLoopNode::se3_ref_callback, this);
  // Publishers
  force_request_pub_ = nh_private_.advertise<auv_msgs::BodyForceRequest>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/force", "/force_bypass"), 5);

  // Timer
  timer_ = nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &InnerLoopNode::timerIterCallback, this);

  // Controller
  if(controller_type_ == "se3_tracker") {
    controller_ = std::make_unique<Se3Tracker>(&nh_, &nh_private_);
    } else {
    ROS_FATAL_STREAM("Unknown controller type: " << controller_type_);
    throw std::runtime_error("Unknown controller type");
  }
}

InnerLoopNode::~InnerLoopNode(){
  force_request_pub_.shutdown();
}



void InnerLoopNode::state_callback(const auv_msgs::NavigationStatus &msg)
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


void InnerLoopNode::se3_ref_callback(const farol_docking::SE3Ref &msg)
{
  controller_->p_d_ = {msg.p.x, msg.p.y, msg.p.z};
  controller_->pd_d_ = {msg.pd.x, msg.pd.y, msg.pd.z};
  controller_->pdd_d_ = {msg.pdd.x,msg.pdd.y,msg.pdd.z};


  Eigen::Quaterniond qd(msg.q.w, msg.q.x, msg.q.y, msg.q.z);
  qd.normalize(); controller_->R_d_ = qd.toRotationMatrix();


  controller_->w_d_ = {msg.wd.x, msg.wd.y, msg.wd.z};
  controller_->wdd_d_ = {msg.wdd.x, msg.wdd.y, msg.wdd.z};


  t_ref_ = msg.header.stamp.toSec();
  // Optional axis disables
  if(msg.disable_axis.size() == 6){
    for(int i=0;i<6;++i) disable_axis_[i] = msg.disable_axis[i];
  }
}


void InnerLoopNode::timerIterCallback(const ros::TimerEvent &)
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
  InnerLoopNode node(&nh, &nh_private);
  ros::spin();
  return 0;
}




// // Constructor
// InnerLoopNode::InnerLoopNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

//   // Parameters
//   p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
//   debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "debug", false);
//   controller_type_ = FarolGimmicks::getParameters<std::string>(nh_private_, "type", "");
//   reference_frame_ = FarolGimmicks::getParameters<std::string>(nh_private_, "reference_frame", "dock");

//   // Subscribers
//   sub_docking_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/docking_state", "docking/state"), 1, &InnerLoopNode::state_callback, this);
//   sub_position_ref_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/ref_position", "position_ref"), 1, &InnerLoopNode::position_ref_callback, this);
//   sub_attitude_ref_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/ref_attitude", "attitude_ref"), 1, &InnerLoopNode::attitude_ref_callback, this);

//   // Publishers
//   force_request_pub_ = nh_private_.advertise<auv_msgs::BodyForceRequest>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/force", "/force_bypass"), 5);
//   // debug_pub_ = nh_private_.advertise<farol_docking::ControllerDebug>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug", "debug"), 5);

//   // Services
//   // ...

//   // Timer
//   timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &InnerLoopNode::timerIterCallback, this);

//   // instatiate the selected controller
//   if (controller_type_ == "se3_tracker") {
//     controller_.reset(new Se3Tracker(&nh_, &nh_private_));
//   // } else if (controller_type_ == "smc_tracker") {
//     // controller_.reset(new SMCTracker(&nh_, &nh_private_));
//   } else {
//     ROS_FATAL_STREAM("Unknown controller type: " << controller_type_);
//     ros::shutdown();
//   }
// }

// // Destructor
// InnerLoopNode::~InnerLoopNode() {

//   // Shutdown publishers
//   force_request_pub_.shutdown();

//   // Shutdown subscribers
//   sub_docking_state_.shutdown();
//   sub_filter_state_.shutdown();
//   sub_position_ref_.shutdown();
//   sub_attitude_ref_.shutdown();
  
//   // Stop timer
//   timer_.stop();

//   // Shutdown node
//   nh_.shutdown();
//   nh_private_.shutdown();
// }


// void InnerLoopNode::state_callback(const auv_msgs::NavigationStatus &msg){
//   // if the message cooresponds to the selected reference frame
//   if(msg.header.frame_id.find("dock") != std::string::npos){
//     controller_->position_ << msg.local_position.x,msg.local_position.y,msg.local_position.z;  
//     controller_->R_ = (Sophus::SO3d::rotZ(msg.local_attitude.yaw)*Sophus::SO3d::rotY(msg.local_attitude.pitch)*Sophus::SO3d::rotX(msg.local_attitude.roll)).matrix(); 
//   }
//   controller_->v_ << msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z; 
//   controller_->w_ <<  msg.orientation_rate.x,msg.orientation_rate.y,msg.orientation_rate.z; 
// }


// void InnerLoopNode::se3_ref_callback(const farol_msgs::SE3Ref::ConstPtr& msg)
// {
//   controller_->p_d_   = Eigen::Vector3d(msg.p.x,  msg.p.y,  msg.p.z);
//   controller_->pd_d_  = Eigen::Vector3d(msg.pd.x, msg.pd.y, msg.pd.z);
//   controller_->pdd_d_ = Eigen::Vector3d(msg.pdd.x,msg.pdd.y,msg.pdd.z);

//   const auto& q = msg.q;
//   Eigen::Quaterniond qd(q.w, q.x, q.y, q.z);
//   qd.normalize();
//   controller_->R_d_ = qd.toRotationMatrix();

//   controller_->w_d_   = Eigen::Vector3d(msg.wd.x,  msg.wd.y,  msg.wd.z);
//   controller_->wdd_d_ = Eigen::Vector3d(msg.wdd.x, msg.wdd.y, msg.wdd.z);

//   t_ref_ = msg.header.stamp.toSec();
//   disable_axis_[0] = msg.disable_axis[0];
//   disable_axis_[1] = msg.disable_axis[1];
//   disable_axis_[2] = msg.disable_axis[2];
//   disable_axis_[3] = msg.disable_axis[3];
//   disable_axis_[4] = msg.disable_axis[4];
//   disable_axis_[5] = msg.disable_axis[5];
// }


// void InnerLoopNode::timerIterCallback(const ros::TimerEvent &ev)
// {
//   double tnow = ros::Time::now().toSec();
//   double Dt   = tnow - last_it_time_;
//   last_it_time_ = tnow;
//   if (first_it_) { first_it_=false; return; }

//   // Make sure both pos & att references have been received recently (you already check t_position_ref_ / t_attitude_ref_)
//   if (tnow - t_ref_ > 0.2) return;

//   controller_->compute_wrench(Dt);                

//   // Publish your existing wrench message
//   force_request_msg_.wrench.force.x  = controller_->force_(0);
//   force_request_msg_.wrench.force.y  = controller_->force_(1);
//   force_request_msg_.wrench.force.z  = controller_->force_(2);
//   force_request_msg_.wrench.torque.x = controller_->torque_(0);
//   force_request_msg_.wrench.torque.y = controller_->torque_(1);
//   force_request_msg_.wrench.torque.z = controller_->torque_(2);
//   wrench_pub_.publish(force_request_msg_);
// }


// // Main
// int main(int argc, char** argv)
// {
//   // Start ROS node:
//   ros::init(argc, argv, "inner_loops"); 
  
//   // Node handlers
//   ros::NodeHandle nh, nh_private("~");

//   // Create the node class which will handle everything through the callbacks
//   InnerLoopNode inner_loops_node(&nh,&nh_private);
//   ros::spin();

//   return 0;
// }
