// Implementation of the OuterLoopNode class
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Handles core filtering logic for docking using exponential smoothing
#include <farol_docking/outer_loop/outer_loop_node.hpp>  

// Constructor
OuterLoopNode::OuterLoopNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  // Parameters
  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "debug", false);

  // Subscribers
  sub_docking_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state", "docking_state"), 10, &OuterLoopNode::state_callback, this);
  sub_inertial_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/filter_state", "/nav/filter/state"), 10, &OuterLoopNode::state_callback, this);
  sub_start_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/enable", "enable"), 10, &OuterLoopNode::start_callback, this);
  sub_flag_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/flag", "flag"), 10, &OuterLoopNode::flag_callback, this);


  // Publishers
  flag_pub_ = nh_private_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/flag", "flag"), 5);
  // debug_pub_ = nh_private_.advertise<farol_docking::ControllerDebug>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug", "debug"), 5);
  surge_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_surge", "ref/surge"), 5);
  sway_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_sway", "ref/sway"), 5);
  i_yaw_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_yaw", "ref/yaw"), 5);
  i_depth_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_depth", "ref/depth"), 5);
  d_position_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_local_pos", "ref/local/position"), 5);
  d_attitude_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_local_attitude", "ref/local/attitude"), 5);
  force_request_pub_ = nh_private_.advertise<auv_msgs::BodyForceRequest>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/thrust_body_request", "/thrust_body_request"), 5);

  // Timer
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &OuterLoopNode::timerIterCallback, this);

}

// Destructor
OuterLoopNode::~OuterLoopNode() {

  // shutdown publishers
  surge_ref_pub_.shutdown();
  sway_ref_pub_.shutdown();
  i_yaw_ref_pub_.shutdown();
  flag_pub_.shutdown();
  d_attitude_pub_.shutdown();
  d_position_pub_.shutdown();
  force_request_pub_.shutdown();
  debug_pub_.shutdown();
  
  // shutdown subscribers
  sub_docking_state_.shutdown();
  sub_inertial_state_.shutdown();
  sub_start_.shutdown();
  sub_flag_.shutdown();

  // stop timer
  timer_.stop();

  // shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}


void OuterLoopNode::state_callback(const auv_msgs::NavigationStatus &msg){
  if(msg.header.frame_id.find("dock") != std::string::npos){
    docking_controller_.docking_pos_ << msg.local_position.x,msg.local_position.y,msg.local_position.z; 
  }else{
    docking_controller_.inertial_pos_ << msg.position.north,msg.position.east,msg.position.depth;
    docking_controller_.body_vel_ << msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z;
  }
}

void OuterLoopNode::start_callback(const std_msgs::Empty &msg){
  flag_ = 10;
  flag_msg_.data = 10;
  flag_pub_.publish(flag_msg_);
  docking_controller_.phase_ ="aproximation";
}

void OuterLoopNode::flag_callback(const std_msgs::Int8 &msg){
  flag_ = msg.data;
  if(flag_ = 0)
    docking_controller_.phase_ ="idle";
  
}


void OuterLoopNode::timerIterCallback(const ros::TimerEvent &event) {
  // compute time interval 
  ros::Time new_time = ros::Time::now();
  double Dt = (new_time - last_update_time_ ).toSec();
  last_update_time_ = new_time;

  //TODO add here a time-out to abort also 
  if( docking_controller_.phase_ == "finished"){
    flag_ = 0;
    flag_msg_.data = 0;
    flag_pub_.publish(flag_msg_);
  }

  // do nothing if the flag is set to 0 (idle mode)
  if(flag_ < 10)
    return;
  
  // Compute references for inner loops
  bool success = docking_controller_.compute(Dt);

  // publish debug message
  if(debug_){ 
    //TODO:ADD some debug message
  }
  // if something failed on computing update do not publish it to the inner-loops
  if(!success)
    return;
  
  
  // publish references for the inner-loops
  if(docking_controller_.output_.frame_id == "inertial_frame"){
    // Surge PID in inertial reference frame
    ref_msg_.data = docking_controller_.output_.data[0];
    surge_ref_pub_.publish(ref_msg_);
    // Yaw PID in inertial reference frame
    ref_msg_.data = docking_controller_.output_.data[1];
    i_yaw_ref_pub_.publish(ref_msg_);
    // depth PID in inertial reference frame
    ref_msg_.data = docking_controller_.output_.data[2];
    i_depth_ref_pub_.publish(ref_msg_);
  }
  else if(docking_controller_.output_.frame_id == "dock_frame"){
    // Relative positions for Position SMC controller
    d_position_ref_msg_.x= docking_controller_.output_.data[0];  // x
    d_position_ref_msg_.y= docking_controller_.output_.data[1];  // y
    d_position_ref_msg_.z= docking_controller_.output_.data[2];  // z
    d_position_pub_.publish(d_position_ref_msg_);

    // Relative Attitude for Attitude SO(3) SMC controller 
    d_attitude_ref_msg_.x= 0;                                    //roll
    d_attitude_ref_msg_.y= 0;                                    //pitch
    d_attitude_ref_msg_.z= docking_controller_.output_.data[5];  //yaw
    d_attitude_pub_.publish(d_attitude_ref_msg_);

    // Surge ref for Farol PID controller 
    ref_msg_.data = docking_controller_.output_.data[6];          // surge
    surge_ref_pub_.publish(ref_msg_);

  } 
  else if(docking_controller_.output_.frame_id == "actuators")
  { // directly publish a force for the thrust allocation
    force_request_msg_.wrench.force.x = docking_controller_.output_.data[0];
    force_request_msg_.wrench.force.y = docking_controller_.output_.data[1];
    force_request_msg_.wrench.force.z = docking_controller_.output_.data[2];
    force_request_msg_.wrench.torque.x = docking_controller_.output_.data[3];
    force_request_msg_.wrench.torque.y = docking_controller_.output_.data[4];
    force_request_msg_.wrench.torque.z = docking_controller_.output_.data[5];
    force_request_pub_.publish(force_request_msg_);
  }
}


// Main
int main(int argc, char** argv)
{
  // Start ROS node:
  ros::init(argc, argv, "outer_loop"); 
  
  // Node handlers
  ros::NodeHandle nh, nh_private("~");

  // Create the node class which will handle everything through the callbacks
  OuterLoopNode outer_loop_node(&nh,&nh_private);
  ros::spin();

  return 0;
}
