#include "DockingVFilter.h"

DockingVFilter::DockingVFilter() {
  initialized_ = false;
  //configure()
  //reset()
}

void DockingVFilter::tune(double Dt, double process_noise, double measurement_noise){
  process_noise_ = process_noise;
  measurement_noise_ = measurement_noise;
  // something else?
}

void DockingVFilter::initialize(double initial_observation){
  state_vec_ = initial_observation;
  state_cov_ = initial_state_cov_;
  initialized_ = true;
}

void DockingVFilter::reset(){
  initialized_=false;
  input_buffer_.clear();
  input_time_buffer_.clear();
  }

bool DockingVFilter::predict()
{
  if(!initialized_)
    return false;
  double t0;
  if(input_buffer_.size()>0)
    t0 = input_time_buffer_.at(0);
  
  // Trapezoidal Integration for state vector prediction
  while(input_buffer_.size()>1)
  {
    state_vec_ += 0.5*(input_buffer_.at(0) + input_buffer_.at(1)) * (input_time_buffer_.at(1) - input_time_buffer_.at(0) );
    input_buffer_.pop_front();
    input_time_buffer_.pop_front();
  }

  // Compute process noise gain G
  if(input_buffer_.size()>0){
    double t1 = input_time_buffer_.at(0);
    double G = t1-t0;
    state_cov_ = state_cov_ + G * process_noise_;
  }
  return true;
}

bool DockingVFilter::update(double measurement){
  outlier_rejected_ = 0;
  innovation_vector_ = measurement - state_vec_;
  innovation_matrix_ = state_cov_ + measurement_noise_;

  
  // Outlier rejection based on mahalanobis distance
  if(usbl_outlier_rejection_){
    mahalanobis_distance_ = std::sqrt(innovation_vector_ / innovation_matrix_ * innovation_vector_);
    outlier_threshold_ = 5; // Chi-squared value for 2 degrees of freedom, alpha = 0.05
    // ROS_WARN_STREAM("Mahalanobis distance = " << mahalanobis_distance_ );
    if (mahalanobis_distance_ > outlier_threshold_) {
      ROS_WARN_STREAM("Docking Vertical: Measurement rejected as outlier (Mahalanobis distance = " << mahalanobis_distance_ << ")");
      outlier_rejected_ = 1;
      return false; 
    }
  }
  
  K_ = state_cov_ / innovation_matrix_;
  state_vec_ = state_vec_ + K_ * innovation_vector_;
  state_cov_  = (1.0 - K_) * state_cov_;
  return true;
}