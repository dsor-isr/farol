// Implementation of the DockingFilter class
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Implements the core logic behind the Docking Filter Algorithm
#include <farol_docking/filter/docking_filter.hpp>  


DockingFilter::DockingFilter()
{
    initialized_ = false;
    measurement_handler_thread_ = std::thread(&DockingFilter::measurement_handler, this);
}

DockingFilter::~DockingFilter()
{
  running_=false;
  measurements_buffer_cond_var_.notify_one();
  if(measurement_handler_thread_.joinable())
    measurement_handler_thread_.join();
}

// for matrix types
void DockingFilter::configure(std::string type, Eigen::MatrixXd noise){
  if(type == "position_process")
    position_filter_.process_noise_ = noise;
  else if(type == "position_measurement")
    position_filter_.measurement_noise_ = noise;
  else if(type == "attitude_process")
    attitude_filter_.process_noise_ = noise;
  else if(type == "attitude_measurement") 
    attitude_filter_.measurement_noise_ = noise;
}
// for usbl rejection configuration
void DockingFilter::configure(std::string type, std::vector<std::string> outlier_rejection_config){
  if(type == "outlier_rejection"){
    if (std::find(outlier_rejection_config.begin(), outlier_rejection_config.end(), "usbl") != outlier_rejection_config.end()){
      position_filter_.output_outlier_rejection_ = true;
      attitude_filter_.output_outlier_rejection_ = true;
    }
    if (std::find(outlier_rejection_config.begin(), outlier_rejection_config.end(), "dvl") != outlier_rejection_config.end())
      position_filter_.input_outlier_rejection_ = true;
    if (std::find(outlier_rejection_config.begin(), outlier_rejection_config.end(), "ahrs") != outlier_rejection_config.end())
      attitude_filter_.input_outlier_rejection_ = true;
  }
}

void DockingFilter::initialize(){
  // do the median filter and get a Sophus 
  Sophus::SE3d new_measurement = extract_se3(median(initializer_buffer_));
  
  // split into R³ and SO(3) and initialize each filter
  position_filter_.initialize(new_measurement.translation());
  attitude_filter_.initialize(new_measurement.so3());
  initialized_=true;
}

void DockingFilter::reset(){
  initialized_=false;
}

Sophus::SE3d DockingFilter::get_state(){
  return Sophus::SE3d(attitude_filter_.state_, position_filter_.state_);
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
          if(initializer_buffer_.size()>=initializer_size_){
            DockingFilter::initialize();
          }
        }
      }else{
      // filter is already initialized -> normally process incoming messages
      if(meas.type=="usbl" && meas.data.value.size() == 6){
        // check that the measurements are valid -> range is ok
        if (std::abs(meas.data.value[0] - meas.data.value[3]) < 2 && meas.data.value[0] > 0.5 && meas.data.value[3] > 0.5){
          // Compute SE(3) object from the two USBL vectors
          Sophus::SE3d measurement = extract_se3(meas.data.value);
          // split into R³ and SO(3) and update each filter
          if(!position_filter_.update(measurement.translation()))
            FAROL_WARN("Update Failed on Docking Position Filter");
          if(!attitude_filter_.update(measurement.so3()))
            FAROL_WARN("Update Failed on Docking Attitude Filter");
        }
      }
      else if(meas.type=="dvl" && meas.data.value.size() == 3){
        // rotate DVL to Dock frame
        Stamped<Eigen::VectorXd> dvl_corrected;
        dvl_corrected.value = attitude_filter_.state_.matrix() * meas.data.value;
        dvl_corrected.stamp = meas.data.stamp;
        if(!position_filter_.predict(dvl_corrected))
          FAROL_WARN("Predict Failed on Docking Position Filter");
      }
      else if(meas.type=="ahrs_rates", meas.data.value.size() ==3){
        if(!attitude_filter_.predict(meas.data))
          FAROL_WARN("Predict Failed on Docking Attitude Filter");
      }else if(meas.type=="ahrs_rates", meas.data.value.size() ==3){
        auv_attitude_ = meas.data.value;
      }else if(meas.type=="dock_attitude", meas.data.value.size() ==3){
        dock_attitude_ = meas.data.value;
      }else
        FAROL_WARN("Invalid measurement type in measurement handler");
      }
    }
  }
}


// FIXME: this only 
Sophus::SE3d DockingFilter::extract_se3(Sophus::Vector6d new_measurement){ 
  Sophus::SO3d R;
  Eigen::Vector3d t;

  // Split into dock and auv RBE: [r, bearing, elevation]
  Eigen::Vector3d rbe_dock = new_measurement.segment<3>(3); // dock sees auv
  Eigen::Vector3d rbe_auv  = new_measurement.segment<3>(0); // auv sees dock

  // compute translatrion component
  double range = (rbe_dock(0) + rbe_auv(0))/2;
  t.x() = range * std::cos(rbe_dock(2)) * std::cos(rbe_dock(1));
  t.y() = range * std::cos(rbe_dock(2)) * std::sin(rbe_dock(1));
  t.z() = range * std::sin(rbe_dock(2));

  // if dock has ahrs compute the rotation matrix based on the diference of rotation matrix
  if(dock_has_ahrs_){
    // represent auv inertial attitude as a rotation matrix from I to B
    Eigen::AngleAxisd yaw_auv(auv_attitude_[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_auv(auv_attitude_[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd roll_auv(auv_attitude_[0], Eigen::Vector3d::UnitX());
    Sophus::SO3d R_auv((yaw_auv * pitch_auv*roll_auv).toRotationMatrix());

    // represent dock inertial attitude as a rotation matrix from I to D
    Eigen::AngleAxisd yaw_dock(dock_attitude_[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_dock(dock_attitude_[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd roll_dock(dock_attitude_[0], Eigen::Vector3d::UnitX());
    Sophus::SO3d R_dock((yaw_dock * pitch_dock * roll_dock).toRotationMatrix());
    
    // compute the rotation matrix from D to B
    R= (R_dock.inverse() * R_auv);
  }
  // FIXME: does not work :( 
  // if dock does not have AHRS, compute orientation purely from usbl measurements
  else
  {
    // Construct SO(3) frame from dock's RBE (X axis points toward AUV)
    Eigen::AngleAxisd yaw_dock(rbe_dock(1), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_dock(rbe_dock(2), Eigen::Vector3d::UnitY());
    Sophus::SO3d R_dock((yaw_dock * pitch_dock).toRotationMatrix());

    // Construct SO(3) frame from AUV's RBE (X axis points toward dock)
    Eigen::AngleAxisd yaw_auv(wrapToPi(rbe_auv(1)+M_PI), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_auv(-rbe_auv(2), Eigen::Vector3d::UnitY());
    Sophus::SO3d R_auv((yaw_auv * pitch_auv).toRotationMatrix());

    // compute rotation from dock frame to 
    R = (R_auv * R_dock.inverse()).inverse();
  }

  return Sophus::SE3d(R, t);
}




// Sophus::SE3d DockingFilter::extract_se3(Sophus::Vector6d new_measurement){ 
//   // Make the ranges equal to the average of dock and auv measurements
//   double range = (new_measurement[0]+new_measurement[3])/2;
//   new_measurement[0] = range;
//   new_measurement[3] = range;

//   // Convert (range,bearing,elevation) -> (x,y,z)
//   Eigen::Vector3d xyz_auv = rbe_to_xyz(new_measurement.segment<3>(0));
//   Eigen::Vector3d xyz_dock = rbe_to_xyz(new_measurement.segment<3>(3));

//   // Compute translation (AUV in Dock frame) 
//   Eigen::Vector3d t = xyz_dock;

//   // Normalize both vectors
//   Eigen::Vector3d v1 = xyz_dock.normalized();    // in Dock frame
//   Eigen::Vector3d v2 = -xyz_auv.normalized();    // negate to align orientation

//   // Compute rotation that aligns v1 to v2
//   double cos_theta = v1.dot(v2);
//   Eigen::Vector3d axis = v1.cross(v2);

//   FAROL_INFO("cos_theta: " << cos_theta);
//   FAROL_INFO("axis: " << axis);

//   Sophus::SO3d R;
//   // Handle the case that the vectors are almost aligned
//   if (axis.norm() < 1e-6) {
//     if (cos_theta > 0.9999) {
//       R = Sophus::SO3d();  // identity
//     } else {
//       Eigen::Vector3d ortho = v1.unitOrthogonal();
//       R = Sophus::SO3d(Eigen::AngleAxisd(M_PI, ortho).toRotationMatrix());
//     }
//   } else {
//     axis.normalize();
//     double theta = std::acos(std::min(std::max(cos_theta, -1.0), 1.0));
//     Eigen::Vector3d so3_vec = theta * axis;
//     R = Sophus::SO3d::exp(so3_vec);
//   }
//   return Sophus::SE3d(R, t);
// }

bool DockingFilter::predict(double time){
  bool ok1 = attitude_filter_.predict(time);
  bool ok2 = position_filter_.predict(time);
  return ok1 && ok2;
}



//#############################################################################################
//           Linear R³ filter
//#############################################################################################

PositionFilter::PositionFilter(){
}

void PositionFilter::initialize(Eigen::Vector3d measurement){
  state_ = measurement;
  // initial covariance is 10% of the initial measurement
  Eigen::Vector3d variance = 0.1*measurement;
  state_cov_ = variance.asDiagonal();
}


// TODO: make this using the proper integration method with the exponential 
// Predict up until a certain measurement
bool PositionFilter::predict(Stamped<Eigen::VectorXd> measurement){
  if (!last_input_measurement_) {
    last_input_measurement_ = measurement;
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


bool PositionFilter::update(Eigen::Vector3d measurement) {

  outlier_rejected_ = 0;
  innovation_vector_ = measurement - state_;
  innovation_matrix_ = state_cov_ + measurement_noise_;
  Eigen::FullPivLU<Eigen::MatrixXd> lu(innovation_matrix_);
  if (!lu.isInvertible()) {
    FAROL_WARN("Docking Position: Innovation Matrix is not invertible");
    return false;
  }

  // Outlier rejection based on mahalanobis distance (Lekkas et al)
  if(output_outlier_rejection_){
    mahalanobis_distance_ = std::sqrt(innovation_vector_.transpose() * innovation_matrix_.inverse() * innovation_vector_);
    if (mahalanobis_distance_ > outlier_threshold_) {
      FAROL_WARN("Docking Position: Measurement rejected as outlier (Mahalanobis distance = " << mahalanobis_distance_ << ")");
      outlier_rejected_ = 1;
      return false; // Skip this update
    }
  }

  K_ = state_cov_ * innovation_matrix_.inverse();
  state_ = state_ + K_ * innovation_vector_;
  state_cov_ = (Eigen::Matrix3d::Identity() - K_) * state_cov_;
  return true;
}





//#############################################################################################
//           Attitude SO(3) filter
//#############################################################################################

AttitudeFilter::AttitudeFilter(){
}

void AttitudeFilter::initialize(Sophus::SO3d measurement){
  state_ = measurement;
  // initial covariance is 10% of the initial measurement
  Eigen::Vector3d variance = 0.1*measurement.log();
  state_cov_ = variance.asDiagonal();
}



// TODO: make this using the proper integration method with the exponential 
bool AttitudeFilter::predict(Stamped<Eigen::VectorXd> measurement){
  if (!last_input_measurement_) {
    last_input_measurement_ = measurement;
    return false;
  }


  return true;
}
bool AttitudeFilter::predict(double time){
  if (!last_input_measurement_) {
    return false;
  }
  return true;
}

bool AttitudeFilter::update(Sophus::SO3d measurement) {
  return true;
}


