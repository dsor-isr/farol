// Implementation of the OuterLoopNode class
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Handles core filtering logic for docking using exponential smoothing
#include <farol_docking/outer_loop/outer_loop_node.hpp>  

// Constructor
OuterLoopNode::OuterLoopNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  // Parameters
  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  acomms_timeout_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_timeout", 20);
  acomms_search_radius_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_search_radius", 10);
  acomms_n_min_fix_ = FarolGimmicks::getParameters<double>(nh_private_, "acomms_n_min_fix", 10);
  
  aproach_dist_ = FarolGimmicks::getParameters<double>(nh_private_, "aproach_dist", 5);
  
  terminal_dist_ = FarolGimmicks::getParameters<double>(nh_private_, "terminal_dist", 0.3);
  terminal_thrust_ = FarolGimmicks::getParameters<double>(nh_private_, "terminal_thrust", 1);
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
  reconfig_srv_ = nh_private_.advertiseService("set_param", &OuterLoopNode::reconfigureParamSrv, this);

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

  {// Pretty, single-line Eigen formatting
  const Eigen::IOFormat rowfmt(3, 0, ", ", ", ", "", "", "[", "]");

  // Helper to print optionals
  auto opt = [](const std::optional<double>& o) -> std::string {
    return o ? std::to_string(*o) : std::string("unset");
  };

  // What source set dock_position_ (optional)
  std::string dock_pos_src = nh_private_.hasParam("dock_utm") ? "utm"
                            : (nh_private_.hasParam("dock_lat_lon") ? "lat_lon" : "unknown");

  ROS_INFO_STREAM(std::fixed << std::setprecision(3)
    << "\n[OuterLoopNode] Parameters"
    << "\n--- Node ---"
    << "\nnode_frequency: "        << node_frequency_
    << "\nstate: "                 << state_
    << "\n--- Acomms ---"
    << "\nacomms_timeout: "        << acomms_timeout_
    << "\nacomms_search_radius: "  << acomms_search_radius_
    << "\nacomms_n_min_fix: "      << acomms_n_min_fix_
    << "\n--- Dock/Mission ---"
    << "\ndock_position (" << dock_pos_src << "): "
    << dock_position_.transpose().format(rowfmt)
    << "\ndock_altitude: "         << opt(dock_altitude_)
    << "\ndock_depth: "            << opt(dock_depth_)
    << "\ndock_heading (deg): "    << opt(dock_heading_)
    << "\nsafe_depth_approach: "   << opt(safe_depth_approach_)
    << "\n--- Geom/Phases ---"
    << "\naproach_dist: "          << aproach_dist_
    << "\nhoming_dist: "           << homing_dist_
    << "\nterminal_dist: "         << terminal_dist_
    << "\ninitial homing_target: " << homing_target_point_.transpose().format(rowfmt)
    << "\n--- Limits ---"
    << "\nu_terminal: "            << u_terminal_
    << "\nv_max_u: "               << v_max_u_
    << "  v_max_v: "               << v_max_v_
    << "\na_max_t: "               << a_max_t_
    << "\nw_max: "                 << w_max_
    << "  a_w_max: "               << a_w_max_
    << "\nr_max: "                 << r_max_
    << "  a_r_max: "               << a_r_max_
    << "\njerk_ratio: "            << jerk_ratio_
  );}

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
    phase_msg_.data = state_;
    docking_state_pub.publish(phase_msg_);
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
    force_request_msg_.wrench.force.x = terminal_thrust_;
    force_request_pub_.publish(force_request_msg_);
  }
  
  
}

bool OuterLoopNode::reconfigureParamSrv(farol_docking::SetGain::Request& req,
                                        farol_docking::SetGain::Response& res)
{
  // Guard: only when idle
  if (state_ != "idle") {
    res.ok = false;
    res.message = "Denied: can only reconfigure when state == 'idle' (current: " + state_ + ")";
    return true;
  }

  auto expect = [&](size_t n) -> bool {
    if (req.values.size() != n) {
      res.ok = false;
      res.message = "Param '" + req.name + "' expects " + std::to_string(n) + " value(s)";
      return false;
    }
    return true;
  };
  auto ok  = [&](const std::string& m){ res.ok = true;  res.message = m; return true; };
  auto bad = [&](const std::string& m){ res.ok = false; res.message = m; return true; };

  auto set_double = [&](double& dst, const std::string& param)->bool{
    if (!expect(1)) return false;
    dst = req.values[0];
    nh_private_.setParam(param, dst);
    return true;
  };
  // auto set_int = [&](int& dst, const std::string& param)->bool{
  //   if (!expect(1)) return false;
  //   dst = static_cast<int>(std::lround(req.values[0]));
  //   nh_private_.setParam(param, dst);
  //   return true;
  // };
  auto set_optional_double = [&](std::optional<double>& dst, const std::string& param)->bool{
    if (req.values.empty()) { dst.reset(); nh_private_.deleteParam(param); return true; }
    if (!expect(1)) return false;
    dst = req.values[0];
    nh_private_.setParam(param, *dst);
    return true;
  };
  auto set_vec2 = [&](Eigen::Vector2d& dst, const std::string& param)->bool{
    if (!expect(2)) return false;
    dst = Eigen::Vector2d(req.values[0], req.values[1]);
    nh_private_.setParam(param, std::vector<double>{dst[0], dst[1]});
    return true;
  };

  std::string k = req.name;
  std::transform(k.begin(), k.end(), k.begin(), ::tolower);

  // Timer rate (safe while idle)
  if (k == "node_frequency" || k == "node_frequency_") {
    if (!expect(1)) return true;
    node_frequency_ = std::max(0.1, req.values[0]);
    nh_private_.setParam("node_frequency", node_frequency_);
    timer_.stop();
    timer_ = nh_.createTimer(ros::Duration(1.0 / node_frequency_),
                             &OuterLoopNode::timerIterCallback, this);
    return ok("node_frequency set to " + std::to_string(node_frequency_));
  }

  // Scalars
  if (k == "terminal_dist" || k == "terminal_dist_") { if(!set_double(terminal_dist_, "terminal_dist_")) return true; return ok("terminal_dist updated"); }
  if (k == "acomms_timeout")            { if(!set_double(acomms_timeout_, "acomms_timeout")) return true; return ok("acomms_timeout updated"); }
  if (k == "acomms_search_radius")      { if(!set_double(acomms_search_radius_, "acomms_search_radius")) return true; return ok("acomms_search_radius updated"); }
  if (k == "acomms_n_min_fix")          { if(!set_double(acomms_n_min_fix_, "acomms_n_min_fix")) return true; return ok("acomms_n_min_fix updated"); }
  if (k == "aproach_dist")              { if(!set_double(aproach_dist_, "aproach_dist")) return true; return ok("aproach_dist updated"); }
  if (k == "homing_dist")               { if(!set_double(homing_dist_, "homing_dist")) return true; return ok("homing_dist updated"); }
  if (k == "u_terminal")                { if(!set_double(u_terminal_, "u_terminal")) return true; return ok("u_terminal updated"); }
  if (k == "v_max_u")                   { if(!set_double(v_max_u_, "v_max_u")) return true; return ok("v_max_u updated"); }
  if (k == "v_max_v")                   { if(!set_double(v_max_v_, "v_max_v")) return true; return ok("v_max_v updated"); }
  if (k == "a_max_t")                   { if(!set_double(a_max_t_, "a_max_t")) return true; return ok("a_max_t updated"); }
  if (k == "w_max")                     { if(!set_double(w_max_, "w_max")) return true; return ok("w_max updated"); }
  if (k == "a_w_max")                   { if(!set_double(a_w_max_, "a_w_max")) return true; return ok("a_w_max updated"); }
  if (k == "r_max")                     { if(!set_double(r_max_, "r_max")) return true; return ok("r_max updated"); }
  if (k == "a_r_max")                   { if(!set_double(a_r_max_, "a_r_max")) return true; return ok("a_r_max updated"); }
  if (k == "jerk_ratio")                { if(!set_double(jerk_ratio_, "jerk_ratio")) return true; return ok("jerk_ratio updated"); }

  // Optionals (pass [] to clear)
  if (k == "dock_altitude")             { if(!set_optional_double(dock_altitude_, "dock_altitude")) return true; return ok(dock_altitude_ ? "dock_altitude set" : "dock_altitude cleared"); }
  if (k == "dock_depth")                { if(!set_optional_double(dock_depth_, "dock_depth")) return true; return ok(dock_depth_ ? "dock_depth set" : "dock_depth cleared"); }
  if (k == "dock_heading")              { if(!set_optional_double(dock_heading_, "dock_heading")) return true; return ok(dock_heading_ ? "dock_heading set" : "dock_heading cleared"); }

  // 2-vectors
  if (k == "dock_lat_lon")              { if(!set_vec2(dock_position_, "dock_lat_lon")) return true; return ok("dock_lat_lon updated"); }
  if (k == "dock_utm")                  { if(!set_vec2(dock_position_, "dock_utm")) return true; return ok("dock_utm updated"); }

  return bad("Unknown param name: '" + req.name + "'");
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
