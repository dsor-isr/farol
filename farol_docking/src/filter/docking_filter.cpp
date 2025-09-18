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
  usbl_pos_dock_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/usbl_pos_dock", "/usbl_pos_dock"), 5);
  usbl_pos_auv_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/usbl_pos_auv", "/usbl_pos_auv"), 5);
  terrain_normal_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/terrain_normal", "/terrain_normal"), 5);
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
    ROS_INFO_STREAM("Process noise is:\n"<<position_filter_->measurement_noise_);
  }
}

void DockingFilter::initialize(double stamp){
  // do the median to account for possible outliers
  Sophus::Vector6d median_meas = median(initializer_buffer_);
  
  // Separate measurements and convert to xyz vectors
  Eigen::Vector3d xyz_auv = rbe_to_xyz(median_meas.segment<3>(0));
  Eigen::Vector3d xyz_dock = rbe_to_xyz(median_meas.segment<3>(3));
  
  // do math to extract the relative yaw
  double r1 = -xyz_dock.dot(xyz_auv);
  double r2 = xyz_dock.cross(xyz_auv)(2);
  // Yaw = atan(r2, r1) [from the slides]
  double yaw = std::atan2(r2, r1);
  Sophus::SO3d R((Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix());
  
  // average the two usbl relative positions (rotating the auv one to the D frame first) and initialize
  position_filter_->initialize(xyz_dock);
  position_filter_->state_at_last_update_ = xyz_dock;
  position_filter_->state_cov_at_last_update_ = position_filter_->state_cov_;
  position_filter_->time_at_last_update_ = stamp;


  // initialize attitude filter with the rotation arround yaw
  attitude_filter_->initialize(R);
  attitude_filter_->state_at_last_update_ = R;
  attitude_filter_->time_at_last_update_ = stamp;


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
          
          // update the attitude filter using both usbl measurments and terrain normal estimate from bottom following
          if(!attitude_filter_->update(meas.data, terrain_normal_))
            ROS_WARN_STREAM("Update Failed on Docking Attitude Filter");
          
          // update using the measurement from the docking station
          aux_vec3_ = rbe_to_xyz(meas.data.value.segment<3>(3));
          aux_vec3_ = dock_usbl_instalation_offset + aux_vec3_ - auv_usbl_instalation_offset; 
          aux_vector3_msg_.x = aux_vec3_[0]; aux_vector3_msg_.y = aux_vec3_[1]; aux_vector3_msg_.z = aux_vec3_[2];
          usbl_pos_dock_pub_.publish(aux_vector3_msg_);
          // ROS_INFO_STREAM("DOCKING::aux_vec3_: "<< std::fixed << std::setprecision(6)<<ros::Time::now().toSec());

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
        // rotate DVL to Dock frame
        Stamped<Eigen::VectorXd> dvl_corrected;
        dvl_corrected.value = attitude_filter_->state_.matrix() * meas.data.value;
        dvl_corrected.stamp = meas.data.stamp;
        if(!position_filter_->predict(dvl_corrected))
          ROS_WARN_STREAM("Predict Failed on Docking Position Filter");
        position_filter_->input_meas_buffer_.emplace_back(dvl_corrected);
        
      }
      else if(meas.type=="ahrs_rates" && meas.data.value.size() ==3){
        if(!attitude_filter_->predict(meas.data))
          ROS_WARN_STREAM("Predict Failed on Docking Attitude Filter");
        attitude_filter_->input_meas_buffer_.emplace_back(meas.data);

      }else if(meas.type=="ahrs_angles"&& meas.data.value.size() ==3){
        auv_attitude_ = meas.data.value;
      }else
        ROS_WARN_STREAM("Invalid measurement type in measurement handler");
      }
    }
  }
}


bool DockingFilter::predict(double time){
  bool ok1 = attitude_filter_->predict(time);
  bool ok2 = position_filter_->predict(time);
  return ok1 && ok2;
}



//#############################################################################################
//           Linear R³ filter
//#############################################################################################

PositionFilter::PositionFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private){
  outlier_rejected_pub_ = nh_private_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/outlier_rejected_usbl_position", "/outlier_rejected_usbl_position"), 5);  
  outlier_test_value_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/outlier_test_value_position", "/outlier_test_value_position"), 5);  
  int8_aux_msg_.data = 1;
}

void PositionFilter::initialize(Eigen::Vector3d measurement){
  state_ = measurement;
  state_cov_ = (0.1*measurement.cwiseAbs()).asDiagonal();    // initial covariance is 10% of the initial measurement
  ROS_INFO_STREAM("Position Filter Initializing with:\nState:\n"<< state_ <<"\nCovariance:\n"<<state_cov_);
}

// TODO: make this using the proper integration method with the exponential 
// Predict up until a certain measurement
bool PositionFilter::predict(Stamped<Eigen::VectorXd> measurement){
  if (!last_input_measurement_) {
    last_input_measurement_ = measurement;
    return false;
  }
  if(last_predict_time_<0){
    last_predict_time_ = measurement.stamp;
    return false;
  }

  // compute time that passed since last predict 
  double Dt = measurement.stamp - last_predict_time_;
  
  // do the standard kalman filter predict for state and covariance
  state_ = state_ + Dt*measurement.value;
  state_cov_ = state_cov_ +  Dt*process_noise_;

  // save the time of last update and the value of last measurement
  last_input_measurement_ = measurement;
  last_predict_time_ = measurement.stamp;

  return true;
}

// predict up until a certain time
bool PositionFilter::predict(double time){
  if (!last_input_measurement_) {
    return false;
  }
  if(last_predict_time_<0){
    return false;
  }

  // if too much time without measurments just stop updating 
  if(time -last_input_measurement_->stamp > 20)
    return false;

  // compute time since last was an update
  double Dt = time - last_predict_time_;
  // do the standard kalman filter predict for state and covariance
  state_ = state_ + Dt*last_input_measurement_->value;
  state_cov_ = state_cov_ +  Dt*process_noise_;

  last_predict_time_=time;

  return true;
}


bool PositionFilter::update(Stamped<Eigen::VectorXd> measurement) {
  state_ = state_at_last_update_;
  state_cov_ = state_cov_at_last_update_;

  double Dt; 
  double time = time_at_last_update_;
  double time_to_update = measurement.stamp - update_delay_;

  int pop_count=0;
  Stamped<Eigen::VectorXd> aux;

  // advance state until correct time to do the update at
  if(!input_meas_buffer_.empty())
    aux = input_meas_buffer_.front();
  while(!input_meas_buffer_.empty() && aux.stamp<time_to_update){
    Dt = aux.stamp-time;
    state_ = state_ + Dt*aux.value;
    state_cov_ = state_cov_ +  Dt*process_noise_;

    time = aux.stamp;
    input_meas_buffer_.pop_front();
    pop_count++;
    
    if(!input_meas_buffer_.empty())
      aux = input_meas_buffer_.front();
  }

  // // ----------------------   perform the update at this time      --------------------------
  // Innovation
  innovation_vector_ = measurement.value - state_;

  // If H != I, use:
  // Eigen::MatrixXd H = ...;
  // innovation_vector_ = measurement.value - H * state_;


  // Innovation covariance S
  // With H = I: S = P + R
  innovation_matrix_ = state_cov_ + measurement_noise_;

  // Prefer Cholesky over LU for SPD matrices
  Eigen::LLT<Eigen::MatrixXd> llt(innovation_matrix_);
  if (llt.info() != Eigen::Success) {
    ROS_WARN_STREAM("Docking Position: Innovation matrix S not SPD (LLT failed).");
    return false;
  }

  // --- Mahalanobis (NIS) gating ---
  float64_aux_msg_.data = innovation_vector_.transpose() * llt.solve(innovation_vector_);
  outlier_test_value_pub_.publish(float64_aux_msg_);
  if (float64_aux_msg_.data > outlier_threshold_) {
    outlier_rejected_pub_.publish(int8_aux_msg_);
    ROS_WARN_STREAM("Docking Position: Outlier rejected. NIS = " << float64_aux_msg_.data);
    return false;  
  }

  // --- Kalman gain ---
  // Compute K = P * S^{-1} via solve (no explicit inverse)
  Eigen::MatrixXd S_inv = llt.solve(Eigen::MatrixXd::Identity(innovation_matrix_.rows(),
                                                              innovation_matrix_.cols()));
  K_ = state_cov_ * S_inv;

  // If H != I, replace with:
  // Eigen::MatrixXd S = H * state_cov_ * H.transpose() + measurement_noise_;
  // llt = Eigen::LLT<Eigen::MatrixXd>(S);
  // if (llt.info()!=Eigen::Success) { ... }
  // K_ = state_cov_ * H.transpose() * llt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

  // --- State update ---
  state_ = state_ + K_ * innovation_vector_;

  // --- Covariance update (Joseph form) ---
  // With H = I:
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_cov_.rows(), state_cov_.cols());
  state_cov_ = (I - K_) * state_cov_ * (I - K_).transpose() + K_ * measurement_noise_ * K_.transpose();

  // If H != I, use: P = (I - K H) P (I - K H)^T + K R K^T


  // -------------------------------------------------------------------------------------

  // from time_to_update till present:
  if(!input_meas_buffer_.empty())
    aux = input_meas_buffer_.front();
  while(!input_meas_buffer_.empty()){
    Dt = aux.stamp-time;
    state_ = state_ + Dt*aux.value;
    state_cov_ = state_cov_ +  Dt*process_noise_;

    time = aux.stamp;
    input_meas_buffer_.pop_front();
    pop_count++;
    if(!input_meas_buffer_.empty())
      aux = input_meas_buffer_.front();
  }

  // save current state and current time
  state_at_last_update_ = state_;
  state_cov_at_last_update_ = state_cov_;
  time_at_last_update_ = time;

  return true;
}


// OLD UPDATE CODE
  // outlier_rejected_ = 0;
  // innovation_vector_ = measurement.value - state_;
  // innovation_matrix_ = state_cov_ + measurement_noise_;
  // Eigen::FullPivLU<Eigen::MatrixXd> lu(innovation_matrix_);
  // if (!lu.isInvertible()) {
  //   FAROL_WARN("Docking Position: Innovation Matrix is not invertible");
  //   return false;
  // }
  // //TODO: add mahalanobis rejetion outliers
  
  // K_ = state_cov_ * innovation_matrix_.inverse();
  // state_ = state_ + K_ * innovation_vector_;
  // state_cov_ = (Eigen::Matrix3d::Identity() - K_) * state_cov_;

// bool PositionFilter::update(Stamped<Eigen::VectorXd> measurement) {

//   outlier_rejected_ = 0;
//   innovation_vector_ = measurement.value - state_;
//   innovation_matrix_ = state_cov_ + measurement_noise_;
//   Eigen::FullPivLU<Eigen::MatrixXd> lu(innovation_matrix_);
//   if (!lu.isInvertible()) {
//     FAROL_WARN("Docking Position: Innovation Matrix is not invertible");
//     return false;
//   }

//   // Outlier rejection based on mahalanobis distance (Lekkas et al)
//   // if(output_outlier_rejection_){
//   //   mahalanobis_distance_ = std::sqrt(innovation_vector_.transpose() * innovation_matrix_.inverse() * innovation_vector_);
//   //   if (mahalanobis_distance_ > outlier_threshold_) {
//   //     FAROL_WARN("Docking Position: Measurement rejected as outlier (Mahalanobis distance = " << mahalanobis_distance_ << ")");
//   //     outlier_rejected_ = 1;
//   //     return false; // Skip this update
//   //   }
//   // }

//   K_ = state_cov_ * innovation_matrix_.inverse();
//   state_ = state_ + K_ * innovation_vector_;
//   state_cov_ = (Eigen::Matrix3d::Identity() - K_) * state_cov_;

//   return true;
// }



//#############################################################################################
//           Attitude SO(3) filter
//#############################################################################################

AttitudeFilter::AttitudeFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private){

  v1_B_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v1_B", "/v1_B"), 5);
  v1_D_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v1_D", "/v1_D"), 5);
  v2_B_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v2_B", "/v2_B"), 5);
  v2_D_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v2_D", "/v2_D"), 5);
  outlier_rejected_pub_ = nh_private_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/outlier_rejected_usbl_attitude", "/outlier_rejected_usbl_attitude"), 5);  
  outlier_test_value_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/outlier_test_value_attitude", "/outlier_test_value_attitude"), 5);  
  int8_aux_msg_.data = 1;
}

void AttitudeFilter::initialize(Sophus::SO3d measurement){
  state_ = measurement;
  // // initial covariance is 10% of the initial measurement
  // Eigen::Vector3d variance = 0.1*measurement.log();
  // state_cov_ = variance.asDiagonal();
}



bool AttitudeFilter::predict(Stamped<Eigen::VectorXd> measurement){
  if (!last_input_measurement_) {
    last_input_measurement_ = measurement;
    return false;
  }
  if(last_predict_time_<0){
    last_predict_time_ = measurement.stamp;
    return false;
  }


  // compute time that passed since last predict 
  double Dt = measurement.stamp - last_predict_time_;
  
  // simple complementary filter in SO3
  state_ = state_ * Sophus::SO3d::exp(Dt*(measurement.value - b_hat_));

  // save the time of last update and the value of last measurement
  last_input_measurement_ = measurement;
  last_predict_time_ = measurement.stamp;
  return true;
}
bool AttitudeFilter::predict(double time){
  if (!last_input_measurement_) {
    return false;
  }

  if(last_predict_time_<0){
    return false;
  }

  // if too much time without measurments just stop updating 
  if(time -last_input_measurement_->stamp > 20)
    return false;

  // compute time since last was an update
  double Dt = time - last_predict_time_;

  // do the standard kalman filter predict for state and covariance
  state_ = state_ * Sophus::SO3d::exp(Dt*(last_input_measurement_->value - b_hat_));

  last_predict_time_=time;
  return true;
}


bool AttitudeFilter::update(Stamped<Eigen::VectorXd> measurement, Eigen::Vector3d terrain_normal_body) {
  state_ = state_at_last_update_;
  double Dt; 
  double time = time_at_last_update_;
  double time_to_update = measurement.stamp - update_delay_;
  int pop_count=0;
  Stamped<Eigen::VectorXd> aux;
  
  // Advance state until correct time to do the update at
  if(!input_meas_buffer_.empty())
    aux = input_meas_buffer_.front();
  while(!input_meas_buffer_.empty() && aux.stamp<time_to_update){
    Dt = aux.stamp-time;
    state_ = state_ * Sophus::SO3d::exp(Dt*(aux.value - b_hat_));
    time = aux.stamp;
    input_meas_buffer_.pop_front();
    pop_count++;

    if(!input_meas_buffer_.empty())
      aux = input_meas_buffer_.front();
  }


  /* ----------------------   Perform the update at this time      -------------------------- */

    // If you don’t have detailed covariances, start isotropic (≈ 3° for LOS)
    const double sigma_v1 = 0.05;  // radians
    const Eigen::Matrix3d Sigma_v1 = (sigma_v1 * sigma_v1) * Eigen::Matrix3d::Identity();

    // Compute the vector directions used for the update
    Eigen::Vector3d v1_B = -1.0 * be_to_xyz(measurement.value[1], measurement.value[2]).normalized();
    Eigen::Vector3d v1_D =        be_to_xyz(measurement.value[4], measurement.value[5]).normalized();
    Eigen::Vector3d v2_B = terrain_normal_body.normalized();      // terrain normal in body
    Eigen::Vector3d v2_D = Eigen::Vector3d::UnitZ();              // dock Z

    // publish for debugging purposes (unchanged)
    v1_B_pub_.publish(toMsg(v1_B)); v1_D_pub_.publish(toMsg(v1_D));
    v2_B_pub_.publish(toMsg(v2_B)); v2_D_pub_.publish(toMsg(v2_D));

    // correction term (v2 always contributes; v1 only if it passes the gate)
    Eigen::Vector3d omega_mes = Eigen::Vector3d::Zero();

    // v2 (terrain normal) — no gating
    omega_mes += k2_ * (v2_B.cross((state_.matrix().transpose() * v2_D).normalized()));

    // v1 (LOS) — χ² gate on S²
    float64_aux_msg_.data = gate_LOS_on_S2(v1_B, v1_D, state_, Sigma_v1);
    outlier_test_value_pub_.publish(float64_aux_msg_);
    if (float64_aux_msg_.data <= outlier_threshold_) {
      omega_mes += k1_ * (v1_B.cross((state_.matrix().transpose() * v1_D).normalized()));
    } else {
      outlier_rejected_pub_.publish(int8_aux_msg_);
      ROS_WARN_STREAM("Attitude: LOS pair rejected by χ² gate (DoF=2). Value: "<<float64_aux_msg_.data);
    }

    // if both got rejected (unlikely here, since v2 always contributes), omega_mes can be small
    if (omega_mes.isZero(1e-12)) {
      return false;  // skip update
    }

    // update state
    state_ = state_ * Sophus::SO3d::exp(kp_ * omega_mes);

    // estimate bias if Ki is not 0
    b_hat_ -= ki_ * omega_mes;
  /* ------------------------------------------------------------------------------------- */

  // Advance state until current time
  while(!input_meas_buffer_.empty()){
    Dt = aux.stamp-time;
    state_ = state_ * Sophus::SO3d::exp(Dt*(aux.value - b_hat_));
    time = aux.stamp;
    input_meas_buffer_.pop_front();
    pop_count++;
    if(!input_meas_buffer_.empty())
      aux = input_meas_buffer_.front();
  }

  // save current state and current time
  state_at_last_update_ = state_;
  time_at_last_update_ = time;

  return true;
}

//OLD UPDATE CODE
  // // Compute the vector directions used for the update
  // Eigen::Vector3d v1_B, v1_D, v2_B, v2_D, omega_mes;
  // v1_B = -1*be_to_xyz(measurement.value[1], measurement.value[2]); 
  // v1_D = be_to_xyz(measurement.value[4], measurement.value[5]);
  // v2_B = terrain_normal_body; // this is the terrain normal, expressed in the b
  // v2_D = Eigen::Vector3d::UnitZ(); // Dock z axis is aligned with terrain normal
  
  // // publish for debugging purposes
  // v1_B_pub_.publish(toMsg(v1_B));
  // v1_D_pub_.publish(toMsg(v1_D));
  // v2_B_pub_.publish(toMsg(v2_B));
  // v2_D_pub_.publish(toMsg(v2_D));

  // // compute correction term
  // omega_mes = Eigen::Vector3d::Zero();
  // omega_mes += k1_* (v1_B.cross(state_.matrix().transpose() * v1_D));
  // omega_mes += k2_* (v2_B.cross(state_.matrix().transpose() * v2_D));

  // // update state
  // state_ = state_ * Sophus::SO3d::exp(kp_ * omega_mes);

  // // estimate bias if Ki is not 0
  // b_hat_ -= ki_ * omega_mes;

// bool AttitudeFilter::update(Stamped<Eigen::VectorXd> measurement, Eigen::Vector3d terrain_normal_body) {
  
//   // Compute the vector directions used for the update
//   Eigen::Vector3d v1_B, v1_D, v2_B, v2_D, omega_mes;
//   v1_B = -1*be_to_xyz(measurement.value[1], measurement.value[2]); 
//   v1_D = be_to_xyz(measurement.value[4], measurement.value[5]);
//   v2_B = terrain_normal_body; // this is the terrain normal, expressed in the b
//   v2_D = Eigen::Vector3d::UnitZ(); // Dock z axis is aligned with terrain normal
  
//   // publish for debugging purposes
//   v1_B_pub_.publish(toMsg(v1_B));di
//   v1_D_pub_.publish(toMsg(v1_D));
//   v2_B_pub_.publish(toMsg(v2_B));
//   v2_D_pub_.publish(toMsg(v2_D));

//   // compute correction term
//   omega_mes = Eigen::Vector3d::Zero();
//   omega_mes += k1_* (v1_B.cross(state_.matrix().transpose() * v1_D));
//   omega_mes += k2_* (v2_B.cross(state_.matrix().transpose() * v2_D));

//   // update state
//   state_ = state_ * Sophus::SO3d::exp(kp_ * omega_mes);

//   // estimate bias if Ki is not 0
//   b_hat_ -= ki_ * omega_mes;

//   return true;
// }


