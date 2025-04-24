#include "DockingHFilter.h"
// https://www.kalmanfilter.net/modeling.html

DockingHFilter::DockingHFilter()
    : A_(Eigen::Matrix4d::Identity()),
      B_(Eigen::MatrixXd::Zero(4, 2)),
      C_(Eigen::MatrixXd::Zero(2, 4)),
      K_(Eigen::MatrixXd::Zero(4, 2)),
      innovation_vector_(Eigen::Vector2d::Zero()),
      innovation_matrix_(Eigen::Matrix2d::Zero())
{
    initialized_ = false;
}


void DockingHFilter::tune(double Dt, Eigen::Matrix4d process_noise, Eigen::Matrix2d measurement_noise){
  process_noise_ = process_noise;
  measurement_noise_ = measurement_noise;
}

void DockingHFilter::initialize(Eigen::Vector2d initial_observation){
  state_vec_ << initial_observation(0), initial_observation(1), 0.0, 0.0;
  state_cov_ = initial_state_cov_;
  initialized_ = true;
}

void DockingHFilter::reset(){
  initialized_=false;
  input_buffer_.clear();
  input_time_buffer_.clear();
}


bool DockingHFilter::predict(double current_yaw){
  if(!initialized_)
    return false;
  double Dt, t0;
  Eigen::Matrix2d R;
  if(input_buffer_.size()>0){
    t0 = input_time_buffer_.at(0);
    R = Rot2D(current_yaw);
  }

  // Trapezoidal Integration for state vector prediction
  while(input_buffer_.size()>1)
  {
    Dt = (input_time_buffer_.at(1) - input_time_buffer_.at(0) );
    A_.block<2, 2>(0, 2) << Dt, 0,
                            0,  Dt;
    B_.block<2, 2>(0, 0) << Dt, 0,
                            0,  Dt;
    state_vec_ = A_ * state_vec_ + B_ * R * 0.5*(input_buffer_.at(0) + input_buffer_.at(1));
    input_buffer_.pop_front();
    input_time_buffer_.pop_front();
  }
  if(input_buffer_.size()>0){
    double t1 = input_time_buffer_.at(0);
    Dt = t1-t0;
    Eigen::Matrix4d G = Eigen::Matrix4d::Identity() * Dt;
    state_cov_ = A_ * state_cov_ * A_.transpose() + G * process_noise_ * G;
  }
  return true;
}



bool DockingHFilter::update(Eigen::Vector2d measurement) {
  outlier_rejected_ = 0;
  C_ << 1, 0, 0, 0,
       0, 1, 0, 0;
  innovation_vector_ = measurement - C_ * state_vec_;
  innovation_matrix_ = C_ * state_cov_ * C_.transpose() + measurement_noise_;
  Eigen::FullPivLU<Eigen::MatrixXd> lu(innovation_matrix_);
  if (!lu.isInvertible()) {
    ROS_ERROR("Docking Horizontal: Innovation Matrix is not invertible");
    return false;
  }

  // Outlier rejection based on mahalanobis distance (Lekkas et al)
  if(usbl_outlier_rejection_){
    mahalanobis_distance_ = std::sqrt(innovation_vector_.transpose() * innovation_matrix_.inverse() * innovation_vector_);
    ROS_WARN_STREAM("HORIZONTAL: " << mahalanobis_distance_);
    if (mahalanobis_distance_ > outlier_threshold_) {
      ROS_WARN_STREAM("Docking Horizontal: Measurement rejected as outlier (Mahalanobis distance = " << mahalanobis_distance_ << ")");
      outlier_rejected_ = 1;
      return false; // Skip this update
    }
  }

  K_ = state_cov_ * C_.transpose() * innovation_matrix_.inverse();
  state_vec_ = state_vec_ + K_ * innovation_vector_;
  state_cov_ = (Eigen::Matrix4d::Identity() - K_ * C_) * state_cov_;
  return true;
}