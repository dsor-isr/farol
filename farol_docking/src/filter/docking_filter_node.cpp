// Implementation of the DockingFilterNode class
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Handles interface between ROS and the docking filter algorithm
#include <farol_docking/filter/docking_filter_node.hpp>  


// Helper function to load a parameter that is a matrix
Eigen::MatrixXd load_matrix_parameter(ros::NodeHandle &_nh, std::string const &parameter_name, Eigen::MatrixXd default_value){
		Eigen::MatrixXd parameter = default_value;
		std::vector<double> temp;
    if (_nh.getParam(parameter_name, temp)) {
        int size = std::sqrt(temp.size()); // Matrix is square, so size = sqrt(vector length)
        if (size * size == temp.size()) {
            Eigen::MatrixXd parameter = Eigen::Map<Eigen::MatrixXd>(temp.data(), size, size);
        } else {
            ROS_ERROR("Invalid process covariance matrix size.");
        }
    } else {
        ROS_ERROR("Failed to load process covariance.");
    }
    return parameter;
}


// Constructor
DockingFilterNode::DockingFilterNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {
  docking_filter_= std::make_unique<DockingFilter>(&nh_,&nh_private_);
  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();
  timer_.start();
  docking_filter_->start(); 
}

// Destructor
DockingFilterNode::~DockingFilterNode() {

  // shutdown publishers
  state_pub_.shutdown();

  // shutdown subscribers
  sub_velocity_.shutdown();
  sub_orientation_.shutdown();
  sub_usbl_fix_.shutdown();
  sub_usbl_accoms_.shutdown();   

  // stop timer
  timer_.stop();

  // shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}


void DockingFilterNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for DockingFilterNode");
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &DockingFilterNode::measurement_callback, this);
  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &DockingFilterNode::measurement_callback, this);
  sub_position_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 10, &DockingFilterNode::measurement_callback, this);
  sub_usbl_fix_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/usbl_fix", "usbl_fix"), 10, &DockingFilterNode::usbl_callback, this);
  sub_usbl_accoms_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/usbl_accoms", "usbl_accoms"), 10, &DockingFilterNode::usbl_callback, this);
  sub_reset_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/reset", "reset"), 10, &DockingFilterNode::reset_callback, this);
  sub_terrain_d_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/terrain_normal", "bottom_following/D"), 10, &DockingFilterNode::terrain_normal_callback, this);
}


void DockingFilterNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for DockingFilterNode");
  state_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "docking/filter/state"), 10);
  body_velocity_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/body_velocity", "docking/filter/debug/body_velocity"), 10);
}


void DockingFilterNode::initializeServices() {
  ROS_INFO("Initializing Services for DockingFilterNode");
  reconfig_numeric_srv_ = nh_private_.advertiseService(
      "reconfigure_param", &DockingFilterNode::reconfigureNumericSrv, this);
}


void DockingFilterNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / node_frequency_), &DockingFilterNode::timerIterCallback, this);
}


void DockingFilterNode::loadParams() {
  ROS_INFO("Load the DockingFilterNode parameters");
  // ROS related parameters
  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "debug", false);
  
  // Algorithm related parameters
  docking_filter_->initializer_size_ = FarolGimmicks::getParameters<int>(nh_private_, "initializer_size", 4);
  docking_filter_->dock_has_ahrs_ = FarolGimmicks::getParameters<bool>(nh_private_, "dock_has_ahrs", false);
  realistify_ = FarolGimmicks::getParameters<bool>(nh_private_, "realistify", false);

  std::vector<double> aux;
  aux = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "dock_usbl_instalation_offset", {});
  docking_filter_->dock_usbl_instalation_offset << aux[0], aux[1], aux[2];
  aux = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "auv_usbl_instalation_offset", {});
  docking_filter_->auv_usbl_instalation_offset << aux[0], aux[1], aux[2];

  // Filter parameters
  docking_filter_->configure("Q_P", FarolGimmicks::getParameters<bool>(nh_private_, "position/process_noise", 1));
  docking_filter_->configure("R_P", FarolGimmicks::getParameters<bool>(nh_private_, "position/measurement_noise", 1));

  // load attitude filter parameters
  docking_filter_->attitude_filter_->k1_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/gains/k1", 0.5);
  docking_filter_->attitude_filter_->k2_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/gains/k2", 0.5);
  docking_filter_->attitude_filter_->kp_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/gains/kp", 1);
  docking_filter_->attitude_filter_->ki_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/gains/ki", 0);

  // delay to apply measuremts because of the roll-back/forward
  docking_filter_->position_filter_->update_delay_ = FarolGimmicks::getParameters<double>(nh_private_, "position/update_delay", 0.0);
  docking_filter_->attitude_filter_->update_delay_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/update_delay", 0.0);
  
  // load outlier rejection config
  aux = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "outlier_rejection", {0.0, 0.0, 0.0});
  if (aux.size() != 3) 
    aux = {0.0, 0.0, 0.0};
  if(aux[0] > 0.1)
    docking_filter_->position_filter_->usbl_outlier_rejection_ = true;
  if(aux[1] > 0.1)
   docking_filter_->attitude_filter_->usbl_outlier_rejection_ = true;
  if(aux[2] > 0.1)
   docking_filter_->position_filter_->dvl_outlier_rejection_ = true;
  
  // threshold for gating on outlier rejection test
  docking_filter_->position_filter_->outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "position/outlier_threshold", 0.0);
  docking_filter_->attitude_filter_->outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/outlier_threshold", 0.0);


  // ---- Summary print ----
  const Eigen::IOFormat rowfmt(3, 0, ", ", ", ", "[", "]");
  // Re-read the vector param just for display (keeps YAML truth if you set via params)
  std::vector<double> outlier_vec = nh_private_.param<std::vector<double>>("outlier_rejection",
                                                                          std::vector<double>{0,0,0});
  if (outlier_vec.size() < 3) outlier_vec.resize(3, 0.0);
  ROS_INFO_STREAM(std::fixed << std::setprecision(3)
    << "\n[DockingFilterNode] Parameters"
    << "\ndebug: "                  << (debug_ ? "true" : "false")
    << "\nitializer_size: "        << docking_filter_->initializer_size_
    << "\ndock_has_ahrs: "          << (docking_filter_->dock_has_ahrs_ ? "true" : "false")
    << "\nauv_usbl_instalation_offset:  "
      << docking_filter_->auv_usbl_instalation_offset.transpose().format(rowfmt)
    << "\ndock_usbl_instalation_offset: "
      << docking_filter_->dock_usbl_instalation_offset.transpose().format(rowfmt)

    << "\n--- Position Parameters ---"
    << "\nQ: " << docking_filter_->position_filter_->process_noise_
    << "\nR: " << docking_filter_->position_filter_->measurement_noise_

    << "\n--- Attitude Gains ---"
    << "\nk1: " << docking_filter_->attitude_filter_->k1_
    << "\nk2: " << docking_filter_->attitude_filter_->k2_
    << "\nkp: " << docking_filter_->attitude_filter_->kp_
    << "\nki: " << docking_filter_->attitude_filter_->ki_

    << "\n--- Update Delays (s) ---"
    << "\nposition/update_delay: "  << docking_filter_->position_filter_->update_delay_
    << "  attitude/update_delay: "  << docking_filter_->attitude_filter_->update_delay_

    << "\n--- Outlier Rejection (switches) ---"
    << "\npos_usbl: " << (docking_filter_->position_filter_->usbl_outlier_rejection_ ? "on" : "off")
    << "  att_usbl: " << (docking_filter_->attitude_filter_->usbl_outlier_rejection_ ? "on" : "off")
    << "  dvl: "      << (docking_filter_->position_filter_->dvl_outlier_rejection_ ? "on" : "off")
    << "\n(rosparam outlier_rejection vec): ["
      << (outlier_vec[0]!=0.0 ? "1" : "0") << ", "
      << (outlier_vec[1]!=0.0 ? "1" : "0") << ", "
      << (outlier_vec[2]!=0.0 ? "1" : "0") << "]"

    << "\n--- Outlier Thresholds ---"
    << "\nposition/outlier_treshold: " << docking_filter_->position_filter_->outlier_threshold_
    << "  attitude/outlier_treshold: " << docking_filter_->attitude_filter_->outlier_threshold_
  );
}


void DockingFilterNode::reset_callback(const std_msgs::Empty &msg){
  docking_filter_->reset();
}


void DockingFilterNode::measurement_callback(const dsor_msgs::Measurement &msg) {
  // Measurements from the AHRS -> extract angular velocities
  if (msg.header.frame_id.find("ahrs") != std::string::npos && msg.value.size() == 6) 
  {
    // drop message if filter not initialized
    if(!docking_filter_->initialized_) 
      return; 
    // send measurement into the docking filter   
    if( docking_filter_->measurements_buffer_.push(Measurement(Eigen::Vector3d(msg.value[3], msg.value[4], msg.value[5]), msg.header.stamp.toSec(), "ahrs_rates")))
        docking_filter_->measurements_buffer_cond_var_.notify_one();
    // no space on buffer, tenso
    else
      ROS_WARN_STREAM("Dropping AHRS measurements. Oh no, not good :(");

    ahrs_velocity_ << msg.value[3],msg.value[4],msg.value[5];
  } 
  // Measurements from the DVL -> extract linear velocities
  else if (msg.header.frame_id.find("dvl") != std::string::npos && msg.value.size() == 3) 
  {
    // if(!docking_filter_->initialized_) // keep only last message if not initialized
      ;//return;

    // added - sign because navquest DVL is stoopid
    dvl_velocity_ << msg.value[0],msg.value[1],msg.value[2];
    // rotate 
    dvl_velocity_ = dvl_velocity_ - ahrs_velocity_.cross(r_dvl_);
    body_velocity_pub_.publish(toMsg(dvl_velocity_));

    // send measurement into the docking filter   
    if(docking_filter_->measurements_buffer_.push(Measurement(dvl_velocity_, msg.header.stamp.toSec(), "dvl")))
      docking_filter_->measurements_buffer_cond_var_.notify_one();
    
    // no space on buffer, tenso
    else
      ROS_WARN_STREAM("Dropping DVL measurements. Oh no, not good :(");
  } 
}

void DockingFilterNode::usbl_callback(const farol_msgs::mUSBLFix &msg) {
  const double now = ros::Time::now().toSec();
  const double W = 0.45; // time window where all usbl messages from the same set must be received

  // 0) Evict stale partials (keep only the most recent window)
  for (int i = 0; i < 4; ++i) {
    if (usbl_state_.test(i) && (now - usbl_times_[i] > W)) {
      usbl_state_.reset(i);
    }
  }

  // 1) Map message -> slot and store
  int slot = -1;
  if (msg.header.frame_id.find("usbl") != std::string::npos) {
    // AUV self USBL
    if (msg.type == 0) { // range
      usbl_set_.segment<1>(0) << msg.range;
      slot = 0;
    } else if (msg.type == 1) { // angles
      usbl_set_.segment<2>(1) << msg.bearing_body, msg.elevation_body;
      slot = 1;
    }
  } else {
    // Dock USBL over acoustics
    if (msg.type == 0) { // range
      usbl_set_.segment<1>(3) << msg.range;
      slot = 2;
    } else if (msg.type == 1) { // angles
      usbl_set_.segment<2>(4) << msg.bearing_body, msg.elevation_body;
      slot = 3;
    }
  }
  if (slot < 0) return;

  usbl_state_.set(slot, true);
  usbl_times_[slot] = now;

  // 2) If full set present, verify window and push
  if (usbl_state_.all()) {
    auto [tmin_it, tmax_it] = std::minmax_element(usbl_times_.begin(), usbl_times_.end());
    const double span = *tmax_it - *tmin_it;

    if (span <= W) {
      const double t_meas = *tmax_it; // latest reception time is the most reliable

      if(realistify_){
          // your existing condition
          if(std::abs(usbl_set_[4]) < M_PI/2.0 || usbl_set_[2] < -0.15){
            usbl_state_.reset();
            return;
          }
          std::array<double,6> z;
          for(int k=0;k<6;++k) z[k] = usbl_set_[k];
          UsblFlags F;
          realistify_usbl(z, rng_, P_, &F);
          if(F.reset) {
            usbl_state_.reset();  // you said you’ll ignore on reset
            return;
          } else {
              for(int k=0;k<6;++k) usbl_set_[k] = z[k]; // commit noisy/outlier values
          }
      }

      if (docking_filter_->measurements_buffer_.push(Measurement(usbl_set_, t_meas, "usbl"))) {
        docking_filter_->measurements_buffer_cond_var_.notify_one();
      } else {
        ROS_WARN_STREAM("Dropping USBL measurements. Buffer full.");
      }
      usbl_state_.reset(); // clear for next cycle
    } else {
      // Mixed cycles: drop the oldest only; keep the most recent partials
      const int idx_old = std::distance(usbl_times_.begin(), tmin_it);
      usbl_state_.reset(idx_old);
      ROS_DEBUG_STREAM("USBL set spans " << span << "s (> " << W << "s). Dropping oldest slot " << idx_old);
    }
  }
}


void DockingFilterNode::terrain_normal_callback(const geometry_msgs::Vector3 &msg){
  docking_filter_->terrain_normal_ << msg.x, msg.y, msg.z;
}

bool DockingFilterNode::reconfigureNumericSrv(farol_docking::SetGain::Request& req,
                                              farol_docking::SetGain::Response& res)
{
  auto expect = [&](size_t n)->bool{
    if (req.values.size() != n) {
      res.ok = false; res.message = "Param '"+req.name+"' expects "
                                   + std::to_string(n) + " value(s)";
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
  auto set_int = [&](int& dst, const std::string& param)->bool{
    if (!expect(1)) return false;
    dst = static_cast<int>(std::lround(req.values[0]));
    nh_private_.setParam(param, dst);
    return true;
  };
  auto set_bool = [&](bool& dst, const std::string& param)->bool{
    if (!expect(1)) return false;
    dst = (req.values[0] != 0.0);
    nh_private_.setParam(param, dst);
    return true;
  };
  auto set_vec3 = [&](Eigen::Vector3d& dst, const std::string& param)->bool{
    if (!expect(3)) return false;
    dst = Eigen::Vector3d(req.values[0], req.values[1], req.values[2]);
    nh_private_.setParam(param, std::vector<double>{dst.x(), dst.y(), dst.z()});
    return true;
  };

  std::string k = req.name;
  std::transform(k.begin(), k.end(), k.begin(), ::tolower);

  // --- Node basics ---
  if (k == "node_frequency" || k == "node_frequency_") {
    if (!expect(1)) return true;
    node_frequency_ = std::max(0.1, req.values[0]);
    nh_private_.setParam("node_frequency", node_frequency_);
    timer_.stop();
    timer_ = nh_.createTimer(ros::Duration(1.0 / node_frequency_),
                             &DockingFilterNode::timerIterCallback, this);
    timer_.start();
    return ok("node_frequency set to " + std::to_string(node_frequency_));
  }
  if (k == "debug") {
    if (!set_bool(debug_, "debug")) return true;
    return ok(std::string("debug=") + (debug_ ? "true" : "false"));
  }

  // --- Interface params you loaded in loadParams() ---
  if (k == "initializer_size") {
    if (!set_int(docking_filter_->initializer_size_, "initializer_size")) return true;
    return ok("initializer_size updated");
  }
  if (k == "dock_has_ahrs") {
    if (!set_bool(docking_filter_->dock_has_ahrs_, "dock_has_ahrs")) return true;
    return ok("dock_has_ahrs updated");
  }
  if (k == "dock_usbl_instalation_offset") {
    if (!set_vec3(docking_filter_->dock_usbl_instalation_offset, "dock_usbl_instalation_offset")) return true;
    return ok("dock_usbl_instalation_offset updated");
  }
  if (k == "auv_usbl_instalation_offset") {
    if (!set_vec3(docking_filter_->auv_usbl_instalation_offset, "auv_usbl_instalation_offset")) return true;
    return ok("auv_usbl_instalation_offset updated");
  }

  if (k == "position/measurement_noise" || k == "R") {
    docking_filter_->configure("R_P", req.values[0]);
    return ok("position/gains/R updated");
  }
  if (k == "position/process_noise" || k == "Q") {
    docking_filter_->configure("Q_P", req.values[0]);
    return ok("position/gains/Q updated");
  }

  // --- Attitude (Mahony-like) gains ---
  if (k == "attitude/gains/k1" || k=="Ku") { if(!set_double(docking_filter_->attitude_filter_->k1_, "attitude/gains/k1")) return true; return ok("attitude/gains/k1 updated"); }
  if (k == "attitude/gains/k2"|| k=="Kb") { if(!set_double(docking_filter_->attitude_filter_->k2_, "attitude/gains/k2")) return true; return ok("attitude/gains/k2 updated"); }
  if (k == "attitude/gains/kp"|| k=="Kp") { if(!set_double(docking_filter_->attitude_filter_->kp_, "attitude/gains/kp")) return true; return ok("attitude/gains/kp updated"); }
  if (k == "attitude/gains/ki"|| k=="Ki") { if(!set_double(docking_filter_->attitude_filter_->ki_, "attitude/gains/ki")) return true; return ok("attitude/gains/ki updated"); }

  // --- Update delays (sec) ---
  if (k == "position/update_delay") { if(!set_double(docking_filter_->position_filter_->update_delay_, "position/update_delay")) return true; return ok("position/update_delay updated"); }
  if (k == "attitude/update_delay") { if(!set_double(docking_filter_->attitude_filter_->update_delay_, "attitude/update_delay")) return true; return ok("attitude/update_delay updated"); }

  // --- Outlier rejections (now numeric/bool) ---
  // Option A: set ALL three at once with a 3-vector [pos_usbl, att_usbl, dvl]
  if (k == "outlier_rejection") {
    if (!expect(3)) return true;
    const bool pos_usbl = (req.values[0] != 0.0);
    const bool att_usbl = (req.values[1] != 0.0);
    const bool dvl      = (req.values[2] != 0.0);
    nh_private_.setParam("outlier_rejection", std::vector<double>{
      pos_usbl?1.0:0.0, att_usbl?1.0:0.0, dvl?1.0:0.0
    });
    docking_filter_->position_filter_->usbl_outlier_rejection_  = pos_usbl;
    docking_filter_->attitude_filter_->usbl_outlier_rejection_  = att_usbl;
    docking_filter_->position_filter_->dvl_outlier_rejection_   = dvl;
    return ok("outlier_rejection vector updated");
  }
  // Option B: set individual entries with a single value
  if (k == "outlier_rejection.pos_usbl") {
    if (!expect(1)) return true;
    const bool v = (req.values[0] != 0.0);
    docking_filter_->position_filter_->usbl_outlier_rejection_ = v;
    // keep param vector in sync if it exists
    std::vector<double> vec = nh_private_.param<std::vector<double>>("outlier_rejection", {0,0,0});
    if (vec.size() < 3) vec.resize(3,0.0);
    vec[0] = v?1.0:0.0;
    nh_private_.setParam("outlier_rejection", vec);
    return ok(std::string("outlier_rejection.pos_usbl=")+(v?"on":"off"));
  }
  if (k == "outlier_rejection.att_usbl") {
    if (!expect(1)) return true;
    const bool v = (req.values[0] != 0.0);
    docking_filter_->attitude_filter_->usbl_outlier_rejection_ = v;
    std::vector<double> vec = nh_private_.param<std::vector<double>>("outlier_rejection", {0,0,0});
    if (vec.size() < 3) vec.resize(3,0.0);
    vec[1] = v?1.0:0.0;
    nh_private_.setParam("outlier_rejection", vec);
    return ok(std::string("outlier_rejection.att_usbl=")+(v?"on":"off"));
  }
  if (k == "outlier_rejection.dvl") {
    if (!expect(1)) return true;
    const bool v = (req.values[0] != 0.0);
    docking_filter_->position_filter_->dvl_outlier_rejection_ = v;
    std::vector<double> vec = nh_private_.param<std::vector<double>>("outlier_rejection", {0,0,0});
    if (vec.size() < 3) vec.resize(3,0.0);
    vec[2] = v?1.0:0.0;
    nh_private_.setParam("outlier_rejection", vec);
    return ok(std::string("outlier_rejection.dvl=")+(v?"on":"off"));
  }

  // --- Outlier χ² thresholds (spelling matches your params: 'threshold') ---
  if (k == "position/outlier_threshold" || k == "position/outlier_threshold") {
    if (!set_double(docking_filter_->position_filter_->outlier_threshold_, "position/outlier_threshold")) return true;
    return ok("position/outlier_threshold updated");
  }
  if (k == "attitude/outlier_threshold" || k == "attitude/outlier_threshold") {
    if (!set_double(docking_filter_->attitude_filter_->outlier_threshold_, "attitude/outlier_threshold")) return true;
    return ok("attitude/outlier_threshold updated");
  }

  return bad("Unknown param name: '" + req.name + "'");
}



void DockingFilterNode::timerIterCallback(const ros::TimerEvent &event) {

  // Proper initialization of the filter using the median of the first 5 measurements
  if(!docking_filter_->initialized_){
    return;
  }

  // Predict the state until current time
  // if(!docking_filter_.predict(event.current_real.toSec())){
  // return;
  // }
  // docking_filter_->predict(event.current_real.toSec());

  state_ = docking_filter_->get_state();

  // publish the estimated state
  state_msg_.header.stamp = ros::Time::now();
  ++state_msg_.header.seq;
  state_msg_.header.frame_id = "mdock0";
  Eigen::Vector3d position = state_.translation();
  Eigen::Quaterniond quaternion = state_.unit_quaternion();
  Eigen::Vector3d rpy = extractRPY(state_.so3());//.matrix().eulerAngles(0, 1, 2);
  Eigen::Vector3d dframe_velocity = state_.so3().matrix().inverse() * dvl_velocity_;
  state_msg_.local_position.x = position[0];
  state_msg_.local_position.y = position[1];
  state_msg_.local_position.z = position[2];
  state_msg_.position_variance.north = docking_filter_->position_filter_->state_cov_(0, 0);
  state_msg_.position_variance.east = docking_filter_->position_filter_->state_cov_(1,1);
  state_msg_.position_variance.depth = docking_filter_->position_filter_->state_cov_(2,2);
  state_msg_.local_orientation.x = quaternion.x();
  state_msg_.local_orientation.y = quaternion.y();
  state_msg_.local_orientation.z = quaternion.z();
  state_msg_.local_orientation.w = quaternion.w();
  state_msg_.local_attitude.roll = 180/M_PI*rpy[0];
  state_msg_.local_attitude.pitch = 180/M_PI*rpy[1];
  state_msg_.local_attitude.yaw = 180/M_PI*rpy[2];
  state_msg_.body_velocity.x = dvl_velocity_[0];
  state_msg_.body_velocity.y = dvl_velocity_[1];
  state_msg_.body_velocity.z = dvl_velocity_[2];
  state_msg_.seafloor_velocity.x = dframe_velocity[0];
  state_msg_.seafloor_velocity.y = dframe_velocity[1];
  state_msg_.seafloor_velocity.z = dframe_velocity[2];
  state_msg_.orientation_rate.x = 180/M_PI*ahrs_velocity_[0];
  state_msg_.orientation_rate.y = 180/M_PI*ahrs_velocity_[1];
  state_msg_.orientation_rate.z = 180/M_PI*ahrs_velocity_[2];
  state_pub_.publish(state_msg_);
  return;
}


// Main  
int main(int argc, char** argv)
{
  // node set up:
  ros::init(argc, argv, "docking_filter_node"); //node name

  // node handle
  ros::NodeHandle nh;

  // private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type DockingFilterNode");

  // instantiate an DockingFilterNode class object and pass in pointers to nodehandle public and private for constructor to use
  DockingFilterNode df(&nh,&nh_private);

  // Going into spin: let the callbacks do all the work
  ros::spin();

  return 0;
}


