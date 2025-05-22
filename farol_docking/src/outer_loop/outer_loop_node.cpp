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
  acomms_timeout_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_timeout", 20);
  acomms_search_radius_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_search_radius", 10);
  acomms_n_min_fix_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_n_min_fix", 10);
  u_terminal_ = FarolGimmicks::getParameters<double>(nh_private_, "u_terminal", 0.15);
  
  double helper;
  std::vector<double> helper_vec;
  max_accl_ << 0.1, 0.1, 0.1;
  if (nh_private_.getParam("max_accl", helper_vec) && helper_vec.size() == 3)
    max_accl_ = Eigen::Vector3d(helper_vec[0], helper_vec[1], helper_vec[2]);

  bool ok1, ok2;
  ok1 = (nh_private_.getParam("dock_lat_lon", helper_vec) && helper_vec.size() == 2);
  if(ok1){
    // TODO: convert latlon to utm x,y
    dock_position_ = Eigen::Vector2d(helper_vec[0], helper_vec[1]);
  }
  ok2 = (nh_private_.getParam("dock_utm", helper_vec) && helper_vec.size() == 2);
  if(ok2){
    dock_position_ = Eigen::Vector2d(helper_vec[0], helper_vec[1]);
  }
  if (!ok1 && !ok2) {
    ROS_ERROR("Missing both 'dock_lat_lon' and 'dock_utm'.");
    ros::shutdown();
    throw std::runtime_error("OuterLoopNode failed to initialize.");
  }

  if(nh_private_.getParam("dock_altitude", helper))
    dock_altitude_ = helper;
  if(nh_private_.getParam("dock_depth", helper))
    dock_depth_ = helper;
  if(nh_private_.getParam("dock_heading", helper))
    dock_heading_ = helper;
  if(nh_private_.getParam("safe_depth_approach", helper))
    safe_depth_approach_ = helper;

  // Subscribers
  sub_docking_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state", "docking_state"), 10, &OuterLoopNode::state_callback, this);
  sub_inertial_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/filter_state", "/nav/filter/state"), 10, &OuterLoopNode::state_callback, this);
  sub_start_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/enable", "enable"), 10, &OuterLoopNode::start_callback, this);
  sub_flag_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/flag", "flag"), 10, &OuterLoopNode::flag_callback, this);
  sub_force_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/force", "force_bypass"), 10, &OuterLoopNode::force_callback, this);


  // Publishers
  flag_pub_ = nh_private_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/flag", "flag"), 5);
  surge_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_surge", "ref/surge"), 5);
  depth_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_depth", "ref/depth"), 5);
  yaw_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_yaw", "ref/yaw"), 5);
  position_pub_ = nh_private_.advertise<farol_docking::Reference3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_position", "ref/position"), 5);
  attitude_pub_ = nh_private_.advertise<farol_docking::Reference3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_attitude", "ref/attitude"), 5);
  force_request_pub_ = nh_private_.advertise<auv_msgs::BodyForceRequest>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/thrust_body_request", "/thrust_body_request"), 5);
  docking_state_pub = nh_private_.advertise<std_msgs::String>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/phase", "/docking_state"), 1);

  // Services
  wp_client = nh_private_.serviceClient<waypoint::sendWpType1>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/waypoint", "/waypoint"));

  // Timer
  timer_ =nh_.createTimer(ros::Duration(1.0/node_frequency_), &OuterLoopNode::timerIterCallback, this);


  phase_msg_.data = state_;
  ROS_INFO_STREAM("mekieeee");
  docking_state_pub.publish(phase_msg_);
  
  if(dock_heading_)
    homing_target_point_ << dock_position_[0] + homing_dist_*cos(dock_heading_.value()/180*M_PI), dock_position_[1] + homing_dist_*sin(dock_heading_.value()/180*M_PI);
  else
    homing_target_point_ << dock_position_[0] + homing_dist_, dock_position_[1];
}

// Destructor
OuterLoopNode::~OuterLoopNode() {

  // shutdown publishers
  surge_ref_pub_.shutdown();
  yaw_ref_pub_.shutdown();
  depth_ref_pub_.shutdown();
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
    got_docking_state_ = true;
    docking_state_ << msg.local_position.x,msg.local_position.y,msg.local_position.z; 
  }else{
    inertial_state_ << msg.position.north,msg.position.east,msg.position.depth;
  }
}

void OuterLoopNode::force_callback(const auv_msgs::BodyForceRequest &msg){
  if (state_== "homing")
    force_request_msg_ = msg;
}

void OuterLoopNode::usbl_callback(const farol_msgs::mUSBLFix &msg){
  time_last_acomms_ = ros::Time::now().toSec();
  n_fixes_++;
}


void OuterLoopNode::start_callback(const std_msgs::Empty &msg){  
  state_ = "approaching";
  phase_msg_.data = state_;
  docking_state_pub.publish(phase_msg_);
  if(dock_heading_){
    // send initial waypoint to go to the dock position
    wp_srv_.request.x = homing_target_point_[0];//dock_position_[0] + homing_dist_*cos(dock_heading_.value()/180*M_PI);
    wp_srv_.request.y = homing_target_point_[1]; //dock_position_[1] + homing_dist_*sin(dock_heading_.value()/180*M_PI);
    wp_srv_.request.yaw = wrapToPi((dock_heading_.value()+180)/180*M_PI);
    wp_client.call(wp_srv_);
    waiting_completion_ = false;
  }else{
    wp_srv_.request.x = homing_target_point_[0];//dock_position_[0] + homing_dist_;
    wp_srv_.request.y = homing_target_point_[1];
    wp_client.call(wp_srv_);
    waiting_completion_ = false;
  }
}

void OuterLoopNode::flag_callback(const std_msgs::Int8 &msg){
  flag_ = msg.data;
  if(state_ == "approaching")
    // reached waypoint
    if(msg.data == 0 && (inertial_state_.segment<2>(0) - homing_target_point_).norm() < 1){
      state_ = "search_acomms";
      phase_msg_.data = state_;
      docking_state_pub.publish(phase_msg_);
      // start path_following of circle around the dock
    }

  if(state_ == "search_acomms" && msg.data == 0 && got_docking_state_ == false){
    // restarts start_path following of circle
  }
}     

void OuterLoopNode::check_state_transition(){
  // reached waypoint and got acomms -> go into homing mode
  if(state_ == "idle")
    return;
  

  if (state_ != "homing" && got_docking_state_ && (inertial_state_.segment<2>(0) - homing_target_point_).norm() < 3){
    state_ = "homing";
    phase_msg_.data = state_;
    docking_state_pub.publish(phase_msg_);
    homing_initial_time_ =ros::Time::now().toSec();
    homing_initial_x_ = docking_state_[0];
    homing_initial_y_ = docking_state_[1];
    homing_initial_x_ = docking_state_[2];
    homing_converging_time_x_ = u_terminal_/(2*max_accl_[0]) - docking_state_[0]/u_terminal_;
    homing_converging_time_y_ = 6*docking_state_[1]/max_accl_[1];
    homing_converging_time_z_ = 6*docking_state_[2]/max_accl_[2];
  }
  
  // lost acomms -> got into search acomms mode
  if(time_last_acomms_ > 0 &&  (ros::Time::now().toSec() - time_last_acomms_) > acomms_timeout_){
    state_ = "search_acomms";
    n_fixes_ =0;
    got_docking_state_ =false;
  }

  // got to close, change to terminal, open loop control

  if(state_ == "homing" && docking_state_.norm() < 0.5){
    state_ = "terminal";
    phase_msg_.data = state_;
    docking_state_pub.publish(phase_msg_);
  }
}

void OuterLoopNode::generate_refs(double time_now, double Dt){
  x_ref_=0; x_ref_dot_=0; x_ref_ddot_=0;
  y_ref_=0; y_ref_dot_=0; y_ref_ddot_=0;
  z_ref_=0; z_ref_dot_=0; z_ref_ddot_=0;
  yaw_ref_=0; yaw_ref_dot_=0; yaw_ref_ddot_=0;

  // time that passed since homing started
  double t = time_now - homing_initial_time_;

  // smooth cubic polinomial trajectory for y
  if (t < homing_converging_time_y_){
    y_ref_ = std::min(0.0, homing_initial_y_*( 2*std::pow(t/homing_converging_time_y_,3) - 3*std::pow(t/homing_converging_time_y_,2) + 1 ));
    y_ref_dot_ = homing_initial_y_ * ( 6*std::pow(t/homing_converging_time_y_,2) - 6 *(t/homing_converging_time_y_) );
    y_ref_ddot_ = homing_initial_y_*6/std::pow(homing_converging_time_y_,2) * (2 *(t/homing_converging_time_y_) -1);
  }

  // smooth cubic polinomial trajectory for z
  if (t < homing_converging_time_z_){
    z_ref_ = std::min(0.0, homing_initial_z_*( 2*std::pow(t/homing_converging_time_z_,3) - 3*std::pow(t/homing_converging_time_z_,2) + 1 ));
    z_ref_dot_ = homing_initial_z_ * ( 6*std::pow(t/homing_converging_time_z_,2) - 6 *(t/homing_converging_time_z_) );
    z_ref_ddot_ = homing_initial_z_*6/std::pow(homing_converging_time_z_,2) * (2 *(t/homing_converging_time_z_) -1);
  }

  // smooth trajetory that goes toward constant speed
  if (t < homing_converging_time_x_) {
    double a = max_accl_[0];
    double t_acc = u_terminal_/ a;
    if (t < t_acc) {
      // Acceleration phase
      x_ref_ = std::min(0.0, homing_initial_x_ + 0.5 * a * t * t);
      x_ref_dot_   = a * t;
      x_ref_ddot_ = a;
    } else {
      // Cruise phase
      double x_acc = 0.5 * a * t_acc * t_acc;
      double t_cruise = t - t_acc;
      
      x_ref_ = std::min(0.0, homing_initial_x_ + x_acc + u_terminal_ * t_cruise);
      x_ref_dot_ = u_terminal_;
      x_ref_ddot_ = 0.0;
    }
  }

  // follows the trajectory for x and y 
  if (std::abs(y_ref_dot_) > 0.2)
    yaw_ref_ = std::atan2(x_ref_dot_, y_ref_dot_)*180/M_PI;
  yaw_ref_dot_ = (x_ref_dot_ * y_ref_ddot_ - y_ref_dot_ * x_ref_ddot_) / (x_ref_dot_ * x_ref_dot_ + y_ref_dot_ * y_ref_dot_);
  yaw_ref_ddot_ = (yaw_ref_dot_ - prev_yaw_ref_dot_) /Dt;
  prev_yaw_ref_dot_ = yaw_ref_dot_;
}

void OuterLoopNode::timerIterCallback(const ros::TimerEvent &event) {
  // idle do nothing

  // compute time interval 
  new_time_ = ros::Time::now().toSec();
  Dt_ = new_time_ - last_update_time_ ;
  if (first_it_){
    first_it_ = false;
    return;
  }

  check_state_transition();
  
  if(state_ == "idle"){
    return;
  }else if(state_=="aproaching"){
    // set the depth reference 
    if(dock_altitude_){
      ref_msg_.data = dock_altitude_.value()+2;
      // floor_dist_pub.publish(ref_msg_);  
    }
    else  if(dock_depth_){
      ref_msg_.data = std::min(0.2,dock_depth_.value()-2);  
      depth_ref_pub_.publish(ref_msg_);
    }
    else if(safe_depth_approach_){
      ref_msg_.data = safe_depth_approach_.value(); 
      depth_ref_pub_.publish(ref_msg_);
    } 
  }else if(state_=="homing"){
    ROS_INFO_STREAM("hdkasjhdakdjsh");
    generate_refs(new_time_, Dt_);
    ref_3d_msg_.value.x = x_ref_;
    ref_3d_msg_.value.y = y_ref_;
    ref_3d_msg_.value.z = z_ref_;
    ref_3d_msg_.value_dot.x = x_ref_dot_;
    ref_3d_msg_.value_dot.y = y_ref_dot_;
    ref_3d_msg_.value_dot.z = z_ref_dot_;
    ref_3d_msg_.value_ddot.x = x_ref_ddot_;
    ref_3d_msg_.value_ddot.y = y_ref_ddot_;
    ref_3d_msg_.value_ddot.z = z_ref_ddot_;
    ref_3d_msg_.disable_axis ={false, false, false};
    position_pub_.publish(ref_3d_msg_);

    ref_3d_msg_.value.x = 0.0;
    ref_3d_msg_.value.y = 0.0;
    ref_3d_msg_.value_dot.x = 0.0;
    ref_3d_msg_.value_dot.y = 0.0;
    ref_3d_msg_.value_ddot.x = 0.0;
    ref_3d_msg_.value_ddot.y = 0.0;
    ref_3d_msg_.value.z = 0.0;//yaw_ref_;
    ref_3d_msg_.value_dot.z = 0.0;//yaw_ref_dot_;
    ref_3d_msg_.value_ddot.z = 0.0;//yaw_ref_ddot_;
    ref_3d_msg_.disable_axis ={true, true, false};
    attitude_pub_.publish(ref_3d_msg_);
    
  }else if(state_ =="terminal"){
    force_request_msg_.disable_axis = {false, false, false, true, true, false};
    force_request_pub_.publish(force_request_msg_);
  }
  
  
  last_update_time_ = new_time_;
}


// Main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "outer_loop_node");
  
  try {
    ros::NodeHandle nh, nh_private("~");
    OuterLoopNode outer_loop_node(&nh,&nh_private);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("Exception in node: %s", e.what());
    return 1;
  }
  return 0;
}
