// Implementation of the DockingFilter class
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Implements the core logic behind the Docking Filter Algorithm
#include <farol_docking/filter/docking_filter.hpp>  


DockingFilter::DockingFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
  position_filter_ = std::make_unique<PositionFilter>(&nh_,&nh_private_);
  attitude_filter_ = std::make_unique<AttitudeFilter>(&nh_,&nh_private_);

  initialized_ = false;
  usbl_pos_dock_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/usbl_pos_dock", "/usbl_pos_dock"), 1);
  usbl_pos_auv_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/usbl_pos_auv", "/usbl_pos_auv"), 1);
  usbl_yaw_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/usbl_yaw", "/usbl_yaw"), 1);
  terrain_normal_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/terrain_normal", "/terrain_normal"), 1);
  dvl_filt_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/dvl_filtered", "/myellow0/docking/filter/dvl_filtered"), 1);


  // DVL mini-KF params (make these ROS params later)
  double q_acc   = FarolGimmicks::getParameters<double>(nh_private_, "dvl_kf/q_acc",   0.01);   // (m/s^2)^2/s
  double r_meas  = FarolGimmicks::getParameters<double>(nh_private_, "dvl_kf/r_meas",  0.04);  // (m/s)^2
  double amax    = FarolGimmicks::getParameters<double>(nh_private_, "dvl_kf/a_max",   0.2);   // m/s^2
  double gate    = FarolGimmicks::getParameters<double>(nh_private_, "dvl_kf/chi2",    4.1); // DoF=3, 95%
  dvl_kf_.configure(q_acc, r_meas, amax, gate);
  dvl_corrected_.value = Eigen::Vector3d::Zero();
}


DockingFilter::~DockingFilter()
{
  running_=false;
  measurements_buffer_cond_var_.notify_one();
  if(measurement_handler_thread_.joinable())
    measurement_handler_thread_.join();
}

void DockingFilter::start()
{
  measurement_handler_thread_ = std::thread(&DockingFilter::measurement_handler, this);
}

// for matrix types
void DockingFilter::configure(std::string type, double noise){
  if(type == "Q_P"){
    position_filter_->process_noise_ = noise*Eigen::Matrix3d::Identity();
    ROS_INFO_STREAM("Process noise is:\n"<<position_filter_->process_noise_);
  }
  else if(type == "R_P"){
    position_filter_->measurement_noise_ = noise*Eigen::Matrix3d::Identity();
    position_filter_->R0_ = position_filter_->measurement_noise_;
    ROS_INFO_STREAM("Process noise is:\n"<<position_filter_->measurement_noise_);
  }
}

void DockingFilter::initialize(double stamp){
  // Compute the median to account for possible outliers
  Sophus::Vector6d median_meas = median(initializer_buffer_);
  
  // Initialize Mahony with yaw correspondent to pitch = roll = 0 which can be computed a priori
  // do math to extract the relative yaw
  double yaw =0;
  if (auto _yaw = yaw_from_two_usbl_rbe(median_meas.segment<3>(0), median_meas.segment<3>(3))){yaw = _yaw.value();}
  Sophus::SO3d R((Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix());
  attitude_filter_->initialize(R);

  // Initialize EKF with dock angles and average of ranges
  // Separate measurements and convert to xyz vectors
  median_meas[3] = (median_meas[0] + median_meas[3])/2;
  position_filter_->initialize(rbe_to_xyz(median_meas.segment<3>(3)));

  initializer_buffer_.clear();
  initialized_=true;
}

void DockingFilter::reset(){
  initialized_=false;
}

Sophus::SE3d DockingFilter::get_state(){
  return Sophus::SE3d(attitude_filter_->state_, position_filter_->state_);
}


void DockingFilter::measurement_handler(){
  while (running_) {
    std::unique_lock<std::mutex> lock(measurements_buffer_mutex_);
    measurements_buffer_cond_var_.wait(lock, [this]() {
      return !measurements_buffer_.empty() || !running_;
    });
    lock.unlock(); // Unlock because buffer access doesn't need lock
    
    // This is what actually handles each new message
    Measurement  meas;
    while (measurements_buffer_.pop(meas)) {
      // if the filter is not yet initialize, it must save only some usbl fixes for initialization
      if(!initialized_ && meas.type=="usbl" && meas.data.value.size() == 6){
        // check that the measurements are valid -> range is ok
        if (std::abs(meas.data.value[0] - meas.data.value[3]) < 2 && meas.data.value[0] > 0.5 && meas.data.value[3] > 0.5){
          // save measurement into initializer buffer
          initializer_buffer_.push_back(meas.data.value);
          // if buffer has already enough measurements for initalization
          if(initializer_buffer_.size()>= static_cast<unsigned long>(initializer_size_)){
            DockingFilter::initialize(meas.data.stamp);
          }
        }
      }else{
      // filter is already initialized -> normally process incoming messages
      if(meas.type=="usbl" && meas.data.value.size() == 6){
        
        // check that the measurements are  valid -> range is ok
        if (std::abs(meas.data.value[0] - meas.data.value[3]) < 2 && meas.data.value[0] > 0.01 && meas.data.value[3] > 0.01){
          
          // do math to extract the relative yaw assuming pitch=roll=0 (for debug only)
          if (auto yaw = yaw_from_two_usbl_rbe(meas.data.value.segment<3>(0), meas.data.value.segment<3>(3))){
            float_aux_msg_.data = *yaw * 180.0/M_PI;
            usbl_yaw_pub_.publish(float_aux_msg_);
          }

          // update the attitude filter using both usbl measurments and terrain normal estimate from bottom following
          if(!attitude_filter_->update(meas.data, Z_D_body_))
            ROS_WARN_STREAM("Update Failed on Docking Attitude Filter");
          
          // update using the measurement from the docking station
          meas.data.value[3] = meas.data.value[0]; // this line to use range from auv and angles from dock
          aux_vec3_ = rbe_to_xyz(meas.data.value.segment<3>(3));
          aux_vec3_ = dock_usbl_instalation_offset + aux_vec3_ - auv_usbl_instalation_offset; 
          aux_vector3_msg_.x = aux_vec3_[0]; aux_vector3_msg_.y = aux_vec3_[1]; aux_vector3_msg_.z = aux_vec3_[2];
          usbl_pos_dock_pub_.publish(aux_vector3_msg_);
          aux_stamped_.value = aux_vec3_;
          aux_stamped_.stamp = meas.data.stamp;

          if(!position_filter_->update(aux_stamped_))
            ROS_WARN_STREAM("Update Failed on Docking Position Filter using dock measurement");

          // update using the measurement from the auv rotated to the body using the matrix
          aux_vec3_ = attitude_filter_->state_.matrix() * -1*rbe_to_xyz(meas.data.value.segment<3>(0));
          aux_vec3_ = dock_usbl_instalation_offset + aux_vec3_ - auv_usbl_instalation_offset; 
          aux_vector3_msg_.x = aux_vec3_[0]; aux_vector3_msg_.y = aux_vec3_[1]; aux_vector3_msg_.z = aux_vec3_[2];
          usbl_pos_auv_pub_.publish(aux_vector3_msg_);
          /* Uncomment to use these updates as well*/
          // aux_stamped_.value = aux_vec3_;
          // aux_stamped_.stamp = meas.data.stamp;
          // if(!position_filter_->update(aux_stamped_))
            // FAROL_WARN("Update Failed on Docking Position Filter using dock measurement");

        }
      }
      else if(meas.type=="dvl" && meas.data.value.size() == 3){

        // Smooth and outlier rejction always runs but only commit if flag active
        Eigen::Vector3d v_smoothed;
        if (!dvl_kf_.step(meas.data, v_smoothed))
          ROS_WARN_STREAM("DVL mini-KF step failed (S not SPD or jitter applied).");
        dvl_filt_pub_.publish(toMsg(v_smoothed));

        // Rotate DVL velocity into Dock frame
        if(dvl_outlier_rejection_){
          dvl_corrected_.value = attitude_filter_->state_.matrix()* v_smoothed;
        }else{
          dvl_corrected_.value = attitude_filter_->state_.matrix() * meas.data.value;
        }
        dvl_corrected_.stamp = meas.data.stamp;

        if(!position_filter_->push_input_and_predict(dvl_corrected_))
          ROS_WARN_STREAM("Predict Failed on Docking Position Filter");

      }
      else if(meas.type=="ahrs_rates" && meas.data.value.size() ==3){
          if(!attitude_filter_->push_input_and_predict(meas.data))
            ROS_WARN_STREAM("Attitude push_input_and_predict failed");
      }else
        ROS_WARN_STREAM("Invalid measurement type in measurement handler");
      }
    }
  }
}



//#############################################################################################
//           Linear R³ filter
//#############################################################################################

PositionFilter::PositionFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private){
  outlier_rejected_pub_ = nh_private_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/outlier_rejected_usbl_position", "/outlier_rejected_usbl_position"), 1);  
  outlier_test_value_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/outlier_test_value_position", "/outlier_test_value_position"), 1);  
  r_scale_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/r_scale", "/r_scale"), 1);  
  k_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/K", "/K"), 1);  
  int8_aux_msg_.data = 1;
}

void PositionFilter::initialize(Eigen::Vector3d measurement){
  state_ = measurement;
  state_cov_ = 0.05*measurement.norm()*Eigen::Matrix3d::Identity();  // (0.1*measurement.cwiseAbs()).asDiagonal();    // initial covariance is 10% of the initial measurement
  ROS_INFO_STREAM("Position Filter Initializing with:\nState:\n"<< state_ <<"\nCovariance:\n"<<state_cov_);
}


bool PositionFilter::push_input_and_predict(const Stamped<Eigen::VectorXd>& meas) {
  // Compute Dt safely (no early returns before we maintain the window & snapshot)
  double Dt = 0.0;
  if (last_predict_time_ >= 0.0) {
    Dt = meas.stamp - last_predict_time_;
    if (Dt < 0.0) Dt = 0.0; // out-of-order guard
  }

  // Predict present to this input time
  if (Dt > 0.0) {
    state_     = state_ + Dt * meas.value;
    state_cov_ = state_cov_ + Dt * process_noise_;
  }
  last_predict_time_      = meas.stamp;

  // Always push the input into the 2 s window
  buf_.push_back(Input{meas.stamp, meas.value /*, Pv if you carry it */});

  // --------- Snapshot initialization (first time only) ----------
  if (snap_time_ < 0.0) {
    // Snapshot is the state at the front of window (which is this sample now)
    snap_time_ = buf_.front().stamp;   // == meas.stamp
    snap_x_    = state_;               // <<-- NON-ZERO: your current predicted state
    snap_P_    = state_cov_;
  }

  // --------- Trim window and advance snapshot forward -----------
  const double cutoff = buf_.back().stamp - window_sec_;

  // Pop whole segments strictly before cutoff
  while (buf_.size() >= 2 && buf_.front().stamp < cutoff && buf_[1].stamp <= cutoff) {
    double dt = buf_[1].stamp - buf_.front().stamp;
    if (dt > 0.0) {
      snap_x_  += dt * buf_.front().u;
      snap_P_  += dt * process_noise_;
      snap_time_ += dt;
    }
    buf_.pop_front();
  }
  // Handle partial first segment crossing the cutoff
  if (buf_.size() >= 2 && buf_.front().stamp < cutoff && buf_[1].stamp > cutoff) {
    double dt = cutoff - buf_.front().stamp;
    if (dt > 0.0) {
      snap_x_  += dt * buf_.front().u;
      snap_P_  += dt * process_noise_;
      snap_time_ += dt;
    }
    buf_.front().stamp = cutoff; // keep remainder in window
  }
  return true;
}



bool PositionFilter::update(Stamped<Eigen::VectorXd> measurement) {
  if (buf_.empty() || snap_time_ < 0.0) return false;

  const double t_u   = measurement.stamp - update_delay_;
  const double t_now = buf_.back().stamp;

  // Still reject if it’s older than the 2 s window
  if (t_u < snap_time_) {
    ROS_WARN("USBL older than window; drop or forward-prop.");
    return false;
  }
  // In practice this should rarely trigger, but keep it as a safety net
  double t_eff = t_u;
  if (t_u > t_now) {
    ROS_WARN_THROTTLE(1.0, "USBL t_u > t_now (%.3f > %.3f). Clamping to t_now.", t_u, t_now);
    t_eff = t_now;
  }
  // --- local rollback to 2s-ago snapshot, then integrate to t_eff ---
  Eigen::Vector3d x = snap_x_;
  Eigen::Matrix3d P = snap_P_;
  double t = snap_time_;

  // Find starting index so we can walk segments
  int j = 0;
  // Ensure we start at the first segment that can advance time
  while (j + 1 < (int)buf_.size() && buf_[j+1].stamp <= t) ++j;
  
  // Advance along segments up to t_eff
  integrate_to(t_eff, x, P, j, t);

  const Eigen::Vector3d x_pre = x;
  const Eigen::Matrix3d P_pre = P;

  // ------------------- USBL update -----------------------------------------
  const Eigen::Vector3d nu = measurement.value - x;

  // 1) Build a *stable* S just for gating.
  //    Use a lightly "faded" P so huge P^- doesn't make NIS artificially tiny.
  //    gamma_gate in (0,1]; 0.5 is a safe, conservative default.
  const double gamma_gate = 0.5;
  Eigen::Matrix3d R_gate  = R0_;                 // fixed nominal R (no adaptation)
  Eigen::Matrix3d S_gate  = gamma_gate * P + R_gate;

  Eigen::LLT<Eigen::Matrix3d> llt_gate(S_gate);
  if (llt_gate.info() != Eigen::Success) { ROS_WARN("S_gate not SPD"); return false; }

  // Compute Normalized Inovation Squared
  double nis = nu.dot( llt_gate.solve(nu) );
  float64_aux_msg_.data = nis; outlier_test_value_pub_.publish(float64_aux_msg_);
  // Gate test on NIS with threshold
  if (usbl_outlier_rejection_ && outlier_reject_cnt_<outlier_reject_max_ && nis > outlier_threshold_) {
    outlier_rejected_pub_.publish(int8_aux_msg_);
    ROS_WARN_STREAM("[Position] Outlier rejected, NIS=" << nis << " > " << outlier_threshold_);
    outlier_reject_cnt_++;
    return false;
  }
  outlier_reject_cnt_=0;
  // Proceed with outlier computations
  Eigen::Matrix3d S_upd = P + R0_;
  Eigen::LLT<Eigen::Matrix3d> llt_upd(S_upd);
  if (llt_upd.info() != Eigen::Success) { ROS_WARN("S_upd not SPD"); return false; }

  Eigen::Matrix3d K = P * llt_upd.solve(Eigen::Matrix3d::Identity());
  k_pub_.publish(toMsg(K.diagonal()));
  
  Eigen::Vector3d dx = K * nu;      
  x = x + dx;

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  P = (I - K) * P * (I - K).transpose() + K * R0_ * K.transpose();
  P.diagonal() = P.diagonal().cwiseMax(Eigen::Vector3d::Constant(p_floor_));

  // Fold the update back into the 2s-snapshot 
  const Eigen::Matrix3d dP = P - P_pre;
  const Eigen::Vector3d dX = x - x_pre;
  snap_x_ += dX;
  snap_P_ += dP;

  // ------------------- end of update block ----------------------------------

  // replay from t_eff to present
  integrate_to(t_now, x, P, j, t);

  // overwrite present
  state_ = x; state_cov_ = P;

  return true;
}


bool PositionFilter::integrate_to(double t_target,Eigen::Vector3d& x,Eigen::Matrix3d& P,int& j,double& t)
{
  if (buf_.empty() || t_target < t) return false;

  const int n = static_cast<int>(buf_.size());

  // Make sure j indexes the active segment for time t: buf_[j].stamp <= t < next_stamp
  while (j + 1 < n && buf_[j+1].stamp <= t) ++j;

  auto seg_u   = [&](int k) -> const Eigen::Vector3d& { return buf_[k].u; };
  auto seg_end = [&](int k) -> double {
    return (k + 1 < n) ? buf_[k+1].stamp : buf_.back().stamp;
  };

  const double eps = 1e-12;

  while (t < t_target - eps && j < n) {
    double end = seg_end(j);
    // advance up to either segment end or target
    double dt = std::min(end, t_target) - t;
    if (dt > eps) {
      x += dt * seg_u(j);
      P += dt * process_noise_;
      // If you carry DVL velocity covariance per segment, add it here:
      // P += (dt*dt) * buf_[j].Pv;
      t += dt;
    }

    // If we exactly hit the segment end, move to the next segment
    if (j + 1 < n && std::abs(t - end) <= eps) {
      ++j;    // continue with next segment's u
    } else {
      break;  // we're at t_target or at the end of the last segment
    }
  }
  return true;
}



//#############################################################################################
//           Attitude SO(3) filter
//#############################################################################################

AttitudeFilter::AttitudeFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private){

  v1_B_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v1_B", "/v1_B"), 1);
  v1_D_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v1_D", "/v1_D"), 1);
  v2_B_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v2_B", "/v2_B"), 1);
  v2_D_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v2_D", "/v2_D"), 1);
  omega_1_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/omega1", "/omega1"), 1);
  omega_2_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/omega2", "/omega2"), 1);
  outlier_rejected_pub_ = nh_private_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/outlier_rejected_usbl_attitude", "/outlier_rejected_usbl_attitude"), 1);  
  outlier_test_value_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/outlier_test_value_attitude", "/outlier_test_value_attitude"), 1);  
  int8_aux_msg_.data = 1;
}

void AttitudeFilter::initialize(Sophus::SO3d measurement){
  state_ = measurement;
}

bool AttitudeFilter::push_input_and_predict(const Stamped<Eigen::VectorXd>& meas)
{
  // meas.value is ω (3x1), meas.stamp is time
  double Dt = 0.0;
  if (last_predict_time_ >= 0.0) {
    Dt = meas.stamp - last_predict_time_;
    if (Dt < 0.0) Dt = 0.0; // guard
  }

  // Predict the "present" state forward to this measurement time
  if (Dt > 0.0) {
    state_ = state_ * Sophus::SO3d::exp(Dt * (meas.value - b_hat_));
  }
  last_predict_time_      = meas.stamp;
  // Push into 2 s window
  buf_.push_back(GyroInput{meas.stamp, meas.value});

  // Initialize snapshot the first time
  if (snap_time_ < 0.0) {
    snap_time_ = meas.stamp;
    snap_R_    = state_;     // snapshot attitude equals current predicted
    snap_b_    = b_hat_;     // snapshot bias equals current bias
  }


  // Trim window to keep only last 2 s, advancing the snapshot to the new front
  const double cutoff = buf_.back().stamp - window_sec_;

  // pop full segments strictly before cutoff
  while (buf_.size() >= 2 && buf_.front().stamp < cutoff && buf_[1].stamp <= cutoff) {
    double dt = buf_[1].stamp - buf_.front().stamp;
    if (dt > 0.0) {
      snap_R_   = snap_R_ * Sophus::SO3d::exp(dt * (buf_.front().w - snap_b_));
      snap_time_ += dt;
    }
    buf_.pop_front();
  }

  // partial segment crossing cutoff
  if (buf_.size() >= 2 && buf_.front().stamp < cutoff && buf_[1].stamp > cutoff) {
    double dt = cutoff - buf_.front().stamp;
    if (dt > 0.0) {
      snap_R_   = snap_R_ * Sophus::SO3d::exp(dt * (buf_.front().w - snap_b_));
      snap_time_ += dt;
    }
    buf_.front().stamp = cutoff; // keep the remainder
  }

  return true;
}

bool AttitudeFilter::integrate_to(double t_target,
                                  Sophus::SO3d& R,
                                  int& j,
                                  double& t,
                                  const Eigen::Vector3d& b) // bias to use during replay
{
  if (buf_.empty() || t_target < t) return false;

  const int n = static_cast<int>(buf_.size());
  // ensure j indexes the active segment for time t
  while (j + 1 < n && buf_[j+1].stamp <= t) ++j;

  auto seg_w   = [&](int k) -> const Eigen::Vector3d& { return buf_[k].w; };
  auto seg_end = [&](int k) -> double {
    return (k + 1 < n) ? buf_[k+1].stamp : buf_.back().stamp;
  };

  const double eps = 1e-12;
  while (t < t_target - eps && j < n) {
    double end = seg_end(j);
    double dt  = std::min(end, t_target) - t;
    if (dt > eps) {
      R = R * Sophus::SO3d::exp(dt * (seg_w(j) - b));
      t += dt;
    }
    if (j + 1 < n && std::abs(t - end) <= eps) {
      ++j;
    } else break;
  }
  return true;
}


bool AttitudeFilter::update(Stamped<Eigen::VectorXd> measurement, Eigen::Vector3d Z_D_in_B)
{
  if (buf_.empty() || snap_time_ < 0.0) return false;

  // Time bookkeeping
  const double t_u   = measurement.stamp - update_delay_;   // when this measurement "belongs"
  const double t_now = buf_.back().stamp;
  if (t_u < snap_time_) {
    ROS_WARN("Attitude USBL older than 2 s window; dropping.");
    return false;
  }
  const double t_eff = std::min(t_u, t_now);

  // Start from the snapshot and replay to t_eff
  Sophus::SO3d R = snap_R_;
  Eigen::Vector3d b = snap_b_;
  double t = snap_time_;
  int j = 0;
  while (j + 1 < (int)buf_.size() && buf_[j+1].stamp <= t) ++j;
  integrate_to(t_eff, R, j, t, b);

  // ---------- Mahony correction AT t_eff (your existing gating logic) ----------
  // Build v's
  Eigen::Vector3d v1_B = (-1.0 * be_to_xyz(measurement.value[1], measurement.value[2])).normalized();
  Eigen::Vector3d v1_D = (      be_to_xyz(measurement.value[4], measurement.value[5])).normalized();
  Eigen::Vector3d v2_B = Z_D_in_B.normalized();
  Eigen::Vector3d v2_D = Eigen::Vector3d::UnitZ();

  v1_B_pub_.publish(toMsg(v1_B)); v1_D_pub_.publish(toMsg(v1_D));
  v2_B_pub_.publish(toMsg(v2_B)); v2_D_pub_.publish(toMsg(v2_D));

  Eigen::Vector3d omega_mes = Eigen::Vector3d::Zero();

  // v2 always contributes
  Eigen::Vector3d omega2 = k2_ * (v2_B.cross((R.matrix().transpose() * v2_D).normalized()));
  omega_2_pub_.publish(toMsg(omega2));
  omega_mes += omega2;

  // v1 (LOS) — χ² gate on S² (unchanged)
  const double sigma_v1 = 0.05; // rad
  const Eigen::Matrix3d Sigma_v1 = (sigma_v1*sigma_v1) * Eigen::Matrix3d::Identity();
  double test = gate_LOS_on_S2(v1_B, v1_D, R, Sigma_v1);
  float64_aux_msg_.data = test; outlier_test_value_pub_.publish(float64_aux_msg_);
  if (usbl_outlier_rejection_ && outlier_reject_cnt_<outlier_reject_max_ && test > outlier_threshold_) {
    outlier_rejected_pub_.publish(int8_aux_msg_);
    ROS_WARN_STREAM("[Attitude] LOS outlier rejected, test="<<test);
    outlier_reject_cnt_++;
    // Even if LOS was rejected, terrain term might be nonzero; if it is ~0, skip.
    if (omega_mes.isZero(1e-12)) return false;
  } else {
    Eigen::Vector3d omega1 = k1_ * (v1_B.cross((R.matrix().transpose() * v1_D).normalized()));
    omega_1_pub_.publish(toMsg(omega1));
    omega_mes += omega1;
    outlier_reject_cnt_=0;
  }

  // If still very small, skip update
  if (omega_mes.isZero(1e-12)) return false;

  // Save pre-update at t_eff
  Sophus::SO3d R_pre = R;
  Eigen::Vector3d b_pre = b;

  // Apply Mahony correction (right-invariant on SO(3))
  R = R * Sophus::SO3d::exp(kp_ * omega_mes);
  b = b - ki_ * omega_mes;

  // ---------- Fold the correction back into the snapshot ----------
  // dR maps snapshot attitude to the corrected attitude at t_eff
  Sophus::SO3d dR = R * R_pre.inverse();
  Eigen::Vector3d db = b - b_pre;

  snap_R_ = dR * snap_R_;  // left-multiply snapshot so future replays include the correction
  snap_b_ += db;           // keep snapshot bias consistent

  // ---------- Replay from t_eff to present with the (possibly updated) bias ----------
  integrate_to(t_now, R, j, t, b);

  // Commit present
  state_ = R;
  b_hat_ = b;

  return true;
}