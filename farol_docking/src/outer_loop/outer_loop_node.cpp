// Implementation of the OuterLoopNode class
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Handles core filtering logic for docking using exponential smoothing
#include <farol_docking/outer_loop/outer_loop_node.hpp>  

// Constructor
OuterLoopNode::OuterLoopNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  // Parameters
  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  terminal_dist_ = FarolGimmicks::getParameters<double>(nh_private_, "terminal_dist_", 0.3);
  acomms_timeout_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_timeout", 20);
  acomms_search_radius_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_search_radius", 10);
  acomms_n_min_fix_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_n_min_fix", 10);
  
  aproach_dist_ = FarolGimmicks::getParameters<double>(nh_private_, "aproach_dist", 5);

  homing_dist_ = FarolGimmicks::getParameters<double>(nh_private_, "homing_dist", 2.5);
  u_terminal_ = FarolGimmicks::getParameters<double>(nh_private_, "u_terminal", 0.05);
  v_max_u_ = FarolGimmicks::getParameters<double>(nh_private_, "v_max_u", 0.2);
  v_max_v_ = FarolGimmicks::getParameters<double>(nh_private_, "v_max_v", 0.2);
  a_max_t_ = FarolGimmicks::getParameters<double>(nh_private_, "a_max_t", 0.5);
  w_max_ = FarolGimmicks::getParameters<double>(nh_private_, "w_max", 0.2);
  a_w_max_ = FarolGimmicks::getParameters<double>(nh_private_, "a_w_max", 0.5);
  r_max_ = FarolGimmicks::getParameters<double>(nh_private_, "r_max", 0.5);
  a_r_max_ = FarolGimmicks::getParameters<double>(nh_private_, "a_r_max", 1.0);
  jerk_ratio_ = FarolGimmicks::getParameters<double>(nh_private_, "jerk_ratio", 0.5);
  

  double helper;
  std::vector<double> helper_vec;
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


  // Publishers
  flag_pub_ = nh_private_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/flag", "flag"), 1);
  surge_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_surge", "ref/surge"), 1);
  depth_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_depth", "ref/depth"), 1);
  yaw_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_yaw", "ref/yaw"), 1);
  force_request_pub_ = nh_private_.advertise<auv_msgs::BodyForceRequest>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/force", "/force_bypass"), 1);
  docking_state_pub = nh_private_.advertise<std_msgs::String>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/phase", "/docking_state"), 1);
  mission_string_pub = nh_private_.advertise<std_msgs::String>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/mission_string", "/mission_string"), 1);
  se3_ref_pub_ = nh_private_.advertise<farol_docking::SE3Ref>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/se3_ref", "/se3_ref"), 1);

  // Services
  wp_client = nh_private_.serviceClient<waypoint::sendWpType1>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/waypoint", "/waypoint"));

  // Timer
  timer_ =nh_.createTimer(ros::Duration(1.0/node_frequency_), &OuterLoopNode::timerIterCallback, this);


  phase_msg_.data = state_;
  docking_state_pub.publish(phase_msg_);
  
  if(dock_heading_)
    homing_target_point_ << dock_position_[0] + aproach_dist_*cos(dock_heading_.value()/180*M_PI), dock_position_[1] + aproach_dist_*sin(dock_heading_.value()/180*M_PI);
  else
    homing_target_point_ << dock_position_[0] + aproach_dist_, dock_position_[1];


  state_ = "idle";
  phase_msg_.data = state_;
  docking_state_pub.publish(phase_msg_);
}

// Destructor
OuterLoopNode::~OuterLoopNode() {

  // shutdown publishers
  surge_ref_pub_.shutdown();
  yaw_ref_pub_.shutdown();
  depth_ref_pub_.shutdown();
  flag_pub_.shutdown();
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
    docking_yaw_ = msg.local_attitude.yaw;
    
    // update the homing target point based on known dock heading
    if (!got_docking_state_){
    homing_target_point_ << dock_position_[0] + aproach_dist_*cos((inertial_yaw_-docking_yaw_)/180*M_PI), dock_position_[1] + aproach_dist_*sin((inertial_yaw_-docking_yaw_)/180*M_PI);
      // if state is approaching or search_acomms we go directly to the homing tarteg point in order to start homing
      if(state_ == "aproaching" || state_=="search_acomms"){
        wp_srv_.request.x = homing_target_point_[0];
        wp_srv_.request.y = homing_target_point_[1];
        wp_srv_.request.yaw = wrapToPi(((inertial_yaw_-docking_yaw_)+180)/180*M_PI);
        wp_client.call(wp_srv_);
      }
    }
    got_docking_state_ =true;

  }else{
    inertial_state_ << msg.position.north,msg.position.east,msg.position.depth;
    inertial_yaw_ = msg.orientation.z;
  }
}


void OuterLoopNode::usbl_callback(const farol_msgs::mUSBLFix &msg){
  // store accoms timing in order to know if we lost acomms and need to go search for acomms
  time_last_acomms_ = ros::Time::now().toSec();
}


void OuterLoopNode::start_callback(const std_msgs::Empty &msg){  
  flag_msg_.data = 10;
  flag_pub_.publish(flag_msg_);
  // this should make it go straight into homing phase, hopefully
  if(got_docking_state_){
    state_ = "skibidi";
    check_state_transition(ros::Time::now().toSec());
    return;
  }

  state_ = "approaching";
  phase_msg_.data = state_;
  docking_state_pub.publish(phase_msg_);
  flag_msg_.data = 11;
  flag_pub_.publish(flag_msg_);

  // here we know the dock heading à priori
  if(dock_heading_){
    // send initial waypoint to go to the dock position
    wp_srv_.request.x = homing_target_point_[0];//dock_position_[0] + aproach_dist_*cos(dock_heading_.value()/180*M_PI);
    wp_srv_.request.y = homing_target_point_[1]; //dock_position_[1] + aproach_dist_*sin(dock_heading_.value()/180*M_PI);
    wp_srv_.request.yaw = wrapToPi((dock_heading_.value()+180)/180*M_PI);
    wp_client.call(wp_srv_);
  }// here we know the dock heading based on usbl
  else if (got_docking_state_){
    wp_srv_.request.x = homing_target_point_[0];//dock_position_[0] + aproach_dist_*cos(dock_heading_.value()/180*M_PI);
    wp_srv_.request.y = homing_target_point_[1]; //dock_position_[1] + aproach_dist_*sin(dock_heading_.value()/180*M_PI);
    wp_srv_.request.yaw = wrapToPi(((inertial_yaw_-docking_yaw_)+180)/180*M_PI);
    wp_client.call(wp_srv_);
  } // here we dont know dock heading
  else{
    wp_srv_.request.x = homing_target_point_[0];//dock_position_[0] + aproach_dist_;
    wp_srv_.request.y = homing_target_point_[1];
    wp_client.call(wp_srv_);
  }
}


void OuterLoopNode::flag_callback(const std_msgs::Int8 &msg){
  flag_ = msg.data;
  if(msg.data == 0){
    state_ = "idle";
    got_docking_state_=false;
  } 
  

  if(state_ == "search_acomms" && msg.data == 4 && !got_docking_state_){
    // restarts start_path following of circle
  }
}     


void OuterLoopNode::check_state_transition(double time_now){
  // reached waypoint and got acomms -> go into homing mode
  if(state_ == "idle")
    return;

  // reached initial waypoint
  if( state_ == "z" && (inertial_state_.segment<2>(0) - homing_target_point_).norm() < 2){
    state_ = "search_acomms";
    phase_msg_.data = state_;
    docking_state_pub.publish(phase_msg_);
    flag_msg_.data = 12;
    flag_pub_.publish(flag_msg_);
    
    // start path_following of circle around the dock
    std::string mission = "3\n";
    // add mission reference point 
    mission += std::to_string(dock_position_[1]) + " " + std::to_string(dock_position_[0]) + "\n";
    // add circle 
    mission += "ARC 0.00 " + std::to_string(aproach_dist_) + " 0.00 0.00 0.00 " + std::to_string(-aproach_dist_) + " 0.30 1 " + std::to_string(aproach_dist_) + " -1\n" ;
    mission += "ARC 0.00 " + std::to_string(-aproach_dist_) + " 0.00 0.00 0.00 " + std::to_string(aproach_dist_) + " 0.30 1 " + std::to_string(aproach_dist_) + " -1\n" ;
    mission += "ARC 0.00 " + std::to_string(aproach_dist_) + " 0.00 0.00 0.00 " + std::to_string(-aproach_dist_) + " 0.30 1 " + std::to_string(aproach_dist_) + " -1\n" ;
    mission_string_msg_.data = mission;
    mission_string_pub.publish(mission_string_msg_);

  }
  
  // if has acomms and is close to target point
  // if(state_ == "approaching" && got_docking_state_ && ((docking_state_.segment<2>(0) - Eigen::Vector2d(-aproach_dist_, 0.0) ).norm() < 4) )
  // or has acomms and was searching for acomms
  // if ((state_ != "homing" && got_docking_state_ && (((inertial_state_.segment<2>(0) - homing_target_point_).norm() < 2) || ((docking_state_.segment<2>(0) - Eigen::Vector2d(-aproach_dist_, 0.0) ).norm() < 4))) ||
  //     (state_ == "search_acomms" && got_docking_state_ && ((docking_state_.segment<2>(0) - Eigen::Vector2d(-aproach_dist_, 0.0) ).norm() < 4) ));
      
  if (state_=="skibidi"){
    // publish flag to signal the start of the docking phase
    flag_msg_.data = 13;
    flag_pub_.publish(flag_msg_);
    // docking state pub
    state_ = "homing";
    phase_msg_.data = state_;
    docking_state_pub.publish(phase_msg_);
    // plan trajectory 
    plan_trajectory();
  }
  
  // lost acomms -> got into search acomms mode
  if(time_last_acomms_ > 0 &&  (ros::Time::now().toSec() - time_last_acomms_) > acomms_timeout_){
    state_ = "search_acomms";
    n_fixes_ =0;
    got_docking_state_ =false;
    flag_msg_.data = 12;
    flag_pub_.publish(flag_msg_);
  }

  // got to close, change to terminal, open loop control
  if( state_!="idle" && state_!="terminal" && got_docking_state_ && docking_state_.norm() < terminal_dist_){
    state_ = "terminal";
    phase_msg_.data = state_;
    docking_state_pub.publish(phase_msg_);
    flag_msg_.data = 13;
    flag_pub_.publish(flag_msg_);
  }
}


void OuterLoopNode::plan_trajectory() {
  traj_planned_ = trajectory_.plan(
      docking_state_[0],
      docking_state_[1],
      docking_state_[2],
      docking_yaw_/180*M_PI,
      homing_dist_,
      u_terminal_,
      v_max_u_, v_max_v_,
      a_max_t_,
      w_max_, a_w_max_,
      r_max_, a_r_max_,
      jerk_ratio_
  );

  if (traj_planned_) {
    homing_initial_time_ = ros::Time::now().toSec();
    ROS_INFO("[OuterLoopNode] Trajectory planned. Total time = %.2f s",
             trajectory_.getTotalTime());
  } else {
    ROS_WARN("[OuterLoopNode] Trajectory planning failed!");
  }
}



bool OuterLoopNode::eval_trajectory(double time_now) {
  if (!traj_planned_) return false;

  const double t = time_now - homing_initial_time_;
  double x,y,z, xd,yd,zd, xdd,ydd,zdd, yaw,r,ar;
  trajectory_.evaluate(t, x,y,z, xd,yd,zd, xdd,ydd,zdd, yaw,r,ar);

  // Publish or use these as before
  se3_ref_msg_.p.x = x;
  se3_ref_msg_.p.y = y;
  se3_ref_msg_.p.z = z;
  se3_ref_msg_.pd.x = xd;
  se3_ref_msg_.pd.y = yd;
  se3_ref_msg_.pd.z = zd;
  se3_ref_msg_.pdd.x = xdd;
  se3_ref_msg_.pdd.y = ydd;
  se3_ref_msg_.pdd.z = zdd;
  se3_ref_msg_.pdd.z = zdd;
  q_aux_.setRPY(0.0,0.0,yaw);
  se3_ref_msg_.q = tf2::toMsg(q_aux_);
  se3_ref_msg_.wd.z = r;
  se3_ref_msg_.wdd.z = ar;
  return true;
}


void OuterLoopNode::timerIterCallback(const ros::TimerEvent &event) {
  // idle do nothing

  // compute time interval 
  new_time_ = ros::Time::now().toSec();
  Dt_ = new_time_ - last_update_time_ ;
  last_update_time_ = new_time_;

  if (first_it_){ first_it_ = false; return;}

  check_state_transition(new_time_);
  
  if(state_ == "idle")
  {
    return;
  }
  else if(state_=="aproaching")
  {
    // set the depth reference 
    if(dock_altitude_)
    {
      ref_msg_.data = dock_altitude_.value()+2;
      // floor_dist_pub.publish(ref_msg_);  
    }
    else  if(dock_depth_)
    {
      ref_msg_.data = std::min(0.2,dock_depth_.value()-2);  
      depth_ref_pub_.publish(ref_msg_);
    }
    else if(safe_depth_approach_)
    {
      ref_msg_.data = safe_depth_approach_.value(); 
      depth_ref_pub_.publish(ref_msg_);
    } 
  }
  else if(state_=="homing")
  {
    if (eval_trajectory(new_time_)){
      se3_ref_msg_.disable_axis = {false, false, false, true, true, false};
      se3_ref_pub_.publish(se3_ref_msg_);
    }  
  }
  else if(state_ =="terminal")
  {
    force_request_msg_.wrench.force.x = 3;
    force_request_pub_.publish(force_request_msg_);
  }
  
  
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
