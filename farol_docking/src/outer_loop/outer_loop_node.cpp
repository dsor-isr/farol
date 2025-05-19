// Implementation of the OuterLoopNode class
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Handles core filtering logic for docking using exponential smoothing
#include <farol_docking/outer_loop/outer_loop_node.hpp>  

// Constructor
OuterLoopNode::OuterLoopNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  // Parameters
  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  homing_dist_ = FarolGimmicks::getParameters<double>(nh_private_, "homing_dist", 5);

  double helper;
  std::vector<double> pos_vec;
  if (nh_private_.getParam("dock_lat_lon", pos_vec) && pos_vec.size() == 2) {
    // convert latlon to utm x,y
    dock_position_ = Eigen::Vector2d(pos_vec[0], pos_vec[1]);
  }
  if (nh_private_.getParam("dock_utm", pos_vec) && pos_vec.size() == 2) {
    dock_position_ = Eigen::Vector2d(pos_vec[0], pos_vec[1]);
  }
  if(nh_private_.getParam("dock_altitude", helper))
    dock_altitude_ = helper;
  if(nh_private_.getParam("dock_depth", helper))
    dock_depth_ = helper;
  if(nh_private_.getParam("dock_heading", helper))
    dock_heading_ = helper;
  
  



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
  position_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_local_pos", "ref/local/position"), 5);
  attitude_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_local_attitude", "ref/local/attitude"), 5);
  force_request_pub_ = nh_private_.advertise<auv_msgs::BodyForceRequest>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/thrust_body_request", "/thrust_body_request"), 5);

  // Services
  wp_client = nh_private_.serviceClient<waypoint::sendWpType1>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/waypoint", "/waypoint"));

  // Timer
  timer_ =nh_.createTimer(ros::Duration(1.0/node_frequency_), &OuterLoopNode::timerIterCallback, this);

}

// Destructor
OuterLoopNode::~OuterLoopNode() {

  // shutdown publishers
  surge_ref_pub_.shutdown();
  sway_ref_pub_.shutdown();
  i_yaw_ref_pub_.shutdown();
  flag_pub_.shutdown();
  attitude_pub_.shutdown();
  position_pub_.shutdown();
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
    docking_state_ << msg.local_position.x,msg.local_position.y,msg.local_position.z; 
  }else{
    inertial_state_ << msg.position.north,msg.position.east,msg.position.depth;
    docking_controller_.body_vel_ << msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z;
  }
}

void OuterLoopNode::start_callback(const std_msgs::Empty &msg){
  flag_ = 10;
  flag_msg_.data = 10;
  flag_pub_.publish(flag_msg_);
  
  state_ =1;
  if(dock_heading_){
    // send initial waypoint to go to the dock position
    wp_srv_.request.x = dock_position_.value()[0] + homing_dist_*cos(dock_heading_.value()/180*M_PI);
    wp_srv_.request.y = dock_position_.value()[1] + homing_dist_*sin(dock_heading_.value()/180*M_PI);
    wp_srv_.request.yaw = wrapToPi((dock_heading_.value()+180)/180*M_PI);
    wp_client.call(wp_srv_);
    waiting_completion_ = true;
  }
  
}

void OuterLoopNode::flag_callback(const std_msgs::Int8 &msg){
  flag_ = msg.data;
}

void OuterLoopNode::usbl_callback(const farol_msgs::mUSBLFix &msg){
  got_acomms_ = true;
  time_last_acomms_ = ros::Time::now().toSec();
}

void OuterLoopNode::state_transition(){
    switch (flag_)
    {
    case 10:
      // reached close to waypoint
      if( (docking_position_.segment<2>(0) - dock_position_.value()).norm() < 2)
        if (got_acomms_)
          // transition to the next phase
          flag_ = 11;
          flag_msg_.data = 11;
          flag_pub_.publish(flag_msg_);
      break;
    
    default:
      break;
    }
  }


void OuterLoopNode::timerIterCallback(const ros::TimerEvent &event) {
  // idle do nothing
  if(flag_<10 && flag_>12)
    return;

  // compute time interval 
  new_time_ = ros::Time::now().toSec();
  Dt_ = new_time_ - last_update_time_ ;
  last_update_time_ = new_time_;
  if (first_it){
    first_it = false;
    return;
  }

  // if 
  if(!(dock_position_ && (dock_depth_ || dock_altitude_))){
    ROS_ERROR_STREAM("Docking Failed: No Docking position was specified. Please specify the aproximate location of the Docking Station");
    return;
  }


  // state machine for diferent parts of docking 
  switch(flag_){
    // Aproxximation Phase
    case 10:
      //if dock heading is known a priori
      if (dock_heading_){
        // publish depth ref using position controller
        if(dock_altitude_)
          ref_3d_msg_.value.z = dock_altitude_.value()+2;  
        else  
          ref_3d_msg_.value.z = std::min(0.0,dock_depth_.value()-2);  
        ref_3d_msg_.disable_axis ={true, true, false};
        position_pub_.publish(ref_3d_msg_);
        break;
      }else
      
     

      break;
    case 11:

      break;
    case 12:
      break;
    default:
      return;
  }



  // Compute references for inner loops
  bool success = docking_controller_.compute(Dt_);

  
  else if(docking_controller_.output_.frame_id == "dock_frame"){
    // Relative positions for Position SMC controller
    ref_3d_msg_.value.x = docking_controller_.output_.data[0];  // x
    ref_3d_msg_.value.y= docking_controller_.output_.data[1];  // y
    ref_3d_msg_.value.z= docking_controller_.output_.data[2];  // z
    ref_3d_msg_.disable_axis ={false, false, false};
    position_pub_.publish(ref_3d_msg_);

    // Relative Attitude for Attitude SO(3) SMC controller 
    ref_3d_msg_.value.z= docking_controller_.output_.data[5];  //yaw
    ref_3d_msg_.disable_axis ={true, true, false};
    attitude_pub_.publish(ref_3d_msg_);

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
