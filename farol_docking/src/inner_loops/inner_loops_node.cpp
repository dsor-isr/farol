// Implementation of the inner loop for docking
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Handles core filtering logic for docking using exponential smoothing
#include <farol_docking/inner_loops/inner_loops_node.hpp>  

// Constructor
InnerLoopNode::InnerLoopNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  // Parameters
  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "debug", false);
  controller_type_ = FarolGimmicks::getParameters<std::string>(nh_private_, "type", "");
  reference_frame_ = FarolGimmicks::getParameters<std::string>(nh_private_, "reference_frame", "dock");

  // Subscribers
  sub_docking_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/docking_state", "docking/state"), 10, &InnerLoopNode::state_callback, this);
  sub_filter_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/filter_state", "filter/state"), 10, &InnerLoopNode::state_callback, this);
  sub_position_ref_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/ref_position", "position_ref"), 10, &InnerLoopNode::position_ref_callback, this);
  sub_attitude_ref_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/ref_attitude", "attitude_ref"), 10, &InnerLoopNode::attitude_ref_callback, this);
  sub_flag_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/flag", "flag"), 10, &InnerLoopNode::flag_callback, this);

  // Publishers
  force_request_pub_ = nh_private_.advertise<auv_msgs::BodyForceRequest>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/force", "/force_bypass"), 5);
  // debug_pub_ = nh_private_.advertise<farol_docking::ControllerDebug>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug", "debug"), 5);

  // Services
  // ...

  // Timer
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &InnerLoopNode::timerIterCallback, this);

  // start controller
  if (controller_type_ == "smc") {
    controller_ = std::make_unique<SMC>(&nh_,&nh_private_);
  } else if (controller_type_ == "pid") {
    controller_ = std::make_unique<PID>(&nh_,&nh_private_);
  } else {
    ROS_ERROR_STREAM("Unknown controller type");
    // InnerLoopNode::~InnerLoopNode();
  }
}

// Destructor
InnerLoopNode::~InnerLoopNode() {

  // Shutdown publishers
  force_request_pub_.shutdown();

  // Shutdown subscribers
  sub_docking_state_.shutdown();
  sub_filter_state_.shutdown();
  sub_position_ref_.shutdown();
  sub_attitude_ref_.shutdown();
  sub_flag_.shutdown();
  
  // Stop timer
  timer_.stop();

  // Shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}


void InnerLoopNode::state_callback(const auv_msgs::NavigationStatus &msg){
  // if the message cooresponds to the selected reference frame
  if(msg.header.frame_id.find(reference_frame_) != std::string::npos){
    controller_->setState(msg);
  }
}

void InnerLoopNode::position_ref_callback(const farol_docking::Reference3 &msg){
  controller_->setReference(Eigen::Vector3d(msg.value.x, msg.value.y, msg.value.z), "position");
  t_position_ref_ = ros::Time::now().toSec();
  disable_axis_[0] = msg.disable_axis[0];
  disable_axis_[1] = msg.disable_axis[1];
  disable_axis_[2] = msg.disable_axis[2];
}
void InnerLoopNode::attitude_ref_callback(const farol_docking::Reference3 &msg){
  controller_->setReference(Eigen::Vector3d(msg.value.x, msg.value.y, msg.value.z), "attitude");
  t_attitude_ref_ = ros::Time::now().toSec();
  disable_axis_[3] = msg.disable_axis[0];
  disable_axis_[4] = msg.disable_axis[1];
  disable_axis_[5] = msg.disable_axis[2];
}

void InnerLoopNode::flag_callback(const std_msgs::Int8 &msg){
  flag_ = msg.data;
}


void InnerLoopNode::timerIterCallback(const ros::TimerEvent &event) {
  // compute time interval 
  double new_time = ros::Time::now().toSec();
  double Dt = new_time - last_it_time_;
  last_it_time_ = new_time;
  
  if (new_time - t_position_ref_ < 0.2){
    controller_->compute_force(Dt);
    force_request_msg_.wrench.force.x = controller_->force_[0];
    force_request_msg_.wrench.force.y = controller_->force_[1];
    force_request_msg_.wrench.force.z = controller_->force_[2];
  }
  else{
    force_request_msg_.wrench.force.x=0;
    force_request_msg_.wrench.force.y=0;
    force_request_msg_.wrench.force.z=0; 
  }

  if (new_time - t_attitude_ref_ < 0.2){
    controller_->compute_torque(Dt);
    force_request_msg_.wrench.torque.x = controller_->torque_[0];
    force_request_msg_.wrench.torque.y = controller_->torque_[1];
    force_request_msg_.wrench.torque.z = controller_->torque_[2];
  }else{
    force_request_msg_.wrench.torque.x=0;
    force_request_msg_.wrench.torque.y=0;
    force_request_msg_.wrench.torque.z=0; 
  }

  if (new_time - t_attitude_ref_ < 0.2 || new_time - t_position_ref_ < 0.2){
    force_request_msg_.disable_axis = {disable_axis_[0], disable_axis_[1], disable_axis_[2], disable_axis_[3], disable_axis_[4], disable_axis_[5]};
    force_request_pub_.publish(force_request_msg_);
  }

}


// Main
int main(int argc, char** argv)
{
  // Start ROS node:
  ros::init(argc, argv, "inner_loops"); 
  
  // Node handlers
  ros::NodeHandle nh, nh_private("~");

  // Create the node class which will handle everything through the callbacks
  InnerLoopNode inner_loops_node(&nh,&nh_private);
  ros::spin();

  return 0;
}
