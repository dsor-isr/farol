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
void DockingFilter::configure(std::string type, Eigen::MatrixXd noise){
  if(type == "position_process")
    position_filter_->process_noise_ = noise;
  else if(type == "position_measurement")
    position_filter_->measurement_noise_ = noise;
  else if(type == "attitude_process")
    ;// attitude_filter_.process_noise_ = noise;
  else if(type == "attitude_measurement")
    ;// attitude_filter_.measurement_noise_ = noise;
}
// for usbl rejection configuration
void DockingFilter::configure(std::string type, std::vector<std::string> outlier_rejection_config){
  if(type == "outlier_rejection"){
    if (std::find(outlier_rejection_config.begin(), outlier_rejection_config.end(), "usbl") != outlier_rejection_config.end()){
      position_filter_->output_outlier_rejection_ = true;
      attitude_filter_->output_outlier_rejection_ = true;
    }
    if (std::find(outlier_rejection_config.begin(), outlier_rejection_config.end(), "dvl") != outlier_rejection_config.end())
      position_filter_->input_outlier_rejection_ = true;
    if (std::find(outlier_rejection_config.begin(), outlier_rejection_config.end(), "ahrs") != outlier_rejection_config.end())
      attitude_filter_->input_outlier_rejection_ = true;
  }
}

void DockingFilter::initialize(){
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

  // initialize attitude filter with the rotation arround yaw
  attitude_filter_->initialize(R);

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
          if(initializer_buffer_.size()>=initializer_size_){
            DockingFilter::initialize();
          }
        }
      }else{
      // filter is already initialized -> normally process incoming messages
      if(meas.type=="usbl" && meas.data.value.size() == 6){
        // check that the measurements are  valid -> range is ok
        if (std::abs(meas.data.value[0] - meas.data.value[3]) < 2 && meas.data.value[0] > 0.01 && meas.data.value[3] > 0.01){
          
          // update the attitude filter using both usbl measurments and terrain normal estimate from bottom following
          if(!attitude_filter_->update(meas.data.value, terrain_normal_))
            FAROL_WARN("Update Failed on Docking Attitude Filter");
            
          
          // update using the measurement from the docking station
          aux_vec3_ = rbe_to_xyz(meas.data.value.segment<3>(3));
          aux_vec3_ = dock_usbl_instalation_offset + aux_vec3_ - auv_usbl_instalation_offset; 
          aux_vector3_msg_.x = aux_vec3_[0]; aux_vector3_msg_.y = aux_vec3_[1]; aux_vector3_msg_.z = aux_vec3_[2];
          usbl_pos_dock_pub_.publish(aux_vector3_msg_);
          if(!position_filter_->update(aux_vec3_))
            FAROL_WARN("Update Failed on Docking Position Filter using dock measurement");

          // update using the measurement from the auv rotated to the body using the matrix
          aux_vec3_ = attitude_filter_->state_.matrix() * -1*rbe_to_xyz(meas.data.value.segment<3>(0));
          aux_vec3_ = dock_usbl_instalation_offset + aux_vec3_ - auv_usbl_instalation_offset; 
          aux_vector3_msg_.x = aux_vec3_[0]; aux_vector3_msg_.y = aux_vec3_[1]; aux_vector3_msg_.z = aux_vec3_[2];
          usbl_pos_auv_pub_.publish(aux_vector3_msg_);
          // if(!position_filter_->update(aux_vec3_))
            // FAROL_WARN("Update Failed on Docking Position Filter using auv measurement");
        }
      }
      else if(meas.type=="dvl" && meas.data.value.size() == 3){
        // rotate DVL to Dock frame
        Stamped<Eigen::VectorXd> dvl_corrected;
        dvl_corrected.value = attitude_filter_->state_.matrix() * meas.data.value;
        dvl_corrected.stamp = meas.data.stamp;
        if(!position_filter_->predict(dvl_corrected))
          FAROL_WARN("Predict Failed on Docking Position Filter");
      }
      else if(meas.type=="ahrs_rates" && meas.data.value.size() ==3){
        if(!attitude_filter_->predict(meas.data))
          FAROL_WARN("Predict Failed on Docking Attitude Filter");
      }else if(meas.type=="ahrs_angles"&& meas.data.value.size() ==3){
        auv_attitude_ = meas.data.value;
      }else if(meas.type=="dock_attitude"&& meas.data.value.size() ==3){
        dock_attitude_ = meas.data.value;
      }else
        FAROL_WARN("Invalid measurement type in measurement handler");
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
  sub_Q_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/Q", "medusa_amarelo_zero/docking/filter/position/R"), 10, &PositionFilter::Q_callback, this);
  sub_R_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/R", "medusa_amarelo_zero/docking/filter/position/Q"), 10, &PositionFilter::R_callback, this);
}

void PositionFilter::initialize(Eigen::Vector3d measurement){
  state_ = measurement;
  // initial covariance is 10% of the initial measurement
  Eigen::Vector3d variance = 0.1*measurement;
  state_cov_ = variance.asDiagonal();
}

void PositionFilter::Q_callback(const std_msgs::Float64 &msg){
  process_noise_ = msg.data*Eigen::Matrix3d::Identity();
}

void PositionFilter::R_callback(const std_msgs::Float64 &msg){
  measurement_noise_ = msg.data*Eigen::Matrix3d::Identity();
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
  // if(output_outlier_rejection_){
  //   mahalanobis_distance_ = std::sqrt(innovation_vector_.transpose() * innovation_matrix_.inverse() * innovation_vector_);
  //   if (mahalanobis_distance_ > outlier_threshold_) {
  //     FAROL_WARN("Docking Position: Measurement rejected as outlier (Mahalanobis distance = " << mahalanobis_distance_ << ")");
  //     outlier_rejected_ = 1;
  //     return false; // Skip this update
  //   }
  // }

  K_ = state_cov_ * innovation_matrix_.inverse();
  state_ = state_ + K_ * innovation_vector_;
  state_cov_ = (Eigen::Matrix3d::Identity() - K_) * state_cov_;

  return true;
}





//#############################################################################################
//           Attitude SO(3) filter
//#############################################################################################

AttitudeFilter::AttitudeFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private){

  v1_B_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v1_B", "/v1_B"), 5);
  v1_D_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v1_D", "/v1_D"), 5);
  v2_B_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v2_B", "/v2_B"), 5);
  v2_D_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/v2_D", "/v2_D"), 5);
  
  sub_kp_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/kp", "filter_attitude_kp"), 10, &AttitudeFilter::kp_callback, this);
  sub_ki_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/ki", "filter_attitude_ki"), 10, &AttitudeFilter::ki_callback, this);
  sub_k1_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/k1", "filter_attitude_k1"), 10, &AttitudeFilter::k1_callback, this);
  sub_k2_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/k2", "filter_attitude_k2"), 10, &AttitudeFilter::k2_callback, this);

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


bool AttitudeFilter::update(Sophus::Vector6d measurement, Eigen::Vector3d terrain_normal_body) {
  
  // Compute the vector directions used for the update
  Eigen::Vector3d v1_B, v1_D, v2_B, v2_D, omega_mes;
  v1_B = -1*be_to_xyz(measurement[1], measurement[2]); 
  v1_D = be_to_xyz(measurement[4], measurement[5]);
  v2_B = terrain_normal_body; // this is the terrain normal, expressed in the b
  v2_D = Eigen::Vector3d::UnitZ(); // Dock z axis is aligned with terrain normal
  
  // publish for debugging purposes
  v1_B_pub_.publish(toMsg(v1_B));
  v1_D_pub_.publish(toMsg(v1_D));
  v2_B_pub_.publish(toMsg(v2_B));
  v2_D_pub_.publish(toMsg(v2_D));

  // ROS_WARN_STREAM("v1_B: " << v1_B);
  // ROS_WARN_STREAM("v1_D: " << v1_D);
  // ROS_WARN_STREAM("R^T * v1_D: " <<  state_.matrix().transpose() * v1_D);
  // ROS_WARN_STREAM("matrix: " << state_.matrix().transpose());
  // ROS_WARN_STREAM("product: " <<v1_B.cross(state_.matrix().transpose() * v1_D));

  // compute correction term
  omega_mes = Eigen::Vector3d::Zero();
  omega_mes += k1_* (v1_B.cross(state_.matrix().transpose() * v1_D));
  omega_mes += k2_* (v2_B.cross(state_.matrix().transpose() * v2_D));

  // update state
  state_ = state_ * Sophus::SO3d::exp(kp_ * omega_mes);

  // estimate bias if Ki is not 0
  b_hat_ -= ki_ * omega_mes;

  return true;
}

void AttitudeFilter::kp_callback(const std_msgs::Float64 &msg){
  kp_ = msg.data;
}

void AttitudeFilter::ki_callback(const std_msgs::Float64 &msg){
  ki_ = msg.data;
}

void AttitudeFilter::k1_callback(const std_msgs::Float64 &msg){
  k1_ = msg.data;
}

void AttitudeFilter::k2_callback(const std_msgs::Float64 &msg){
  k2_ = msg.data;
}






