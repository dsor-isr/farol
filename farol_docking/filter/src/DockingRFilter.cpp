#include "DockingRFilter.h"

DockingRFilter::DockingRFilter() {
  initialized_ = false;
  //configure()
  //reset()
}

void DockingRFilter::tune(double Dt, double process_noise, double measurement_noise){
  process_noise_ = process_noise;
  measurement_noise_ = measurement_noise;
  // something else?
}

void DockingRFilter::initialize(double initial_observation){
  state_vec_ = initial_observation;
  state_cov_ = initial_state_cov_;
  initialized_ = true;
}

void DockingRFilter::reset(){
  initialized_=false;
  input_buffer_.clear();
  input_time_buffer_.clear();
  }

bool DockingRFilter::predict()
{
  if(!initialized_)
    return false;
  double t0=0;
  if(input_buffer_.size()>0){
    t0 = input_time_buffer_.at(0);
  }
  // Trapezoidal Integration for state vector prediction
  while(input_buffer_.size()>1)
  {
    // ROS_INFO("%lf", (input_time_buffer_.at(1) - input_time_buffer_.at(0) ));
    state_vec_ += 0.5*(input_buffer_.at(0) + input_buffer_.at(1)) * (input_time_buffer_.at(1) - input_time_buffer_.at(0) );
    input_buffer_.pop_front();
    input_time_buffer_.pop_front();
  }
  state_vec_ = wrapToPi(state_vec_);

  // Compute process noise gain G
  if(input_buffer_.size()>0){
    double t1 = input_time_buffer_.at(0);
    double G = t1-t0;
    state_cov_ = state_cov_ + G * process_noise_;
  }
  return true;
}

bool DockingRFilter::update(double measurement){
  outlier_rejected_ = 0;
  innovation_vector_ = wrapToPi(measurement - state_vec_);
  innovation_matrix_ = state_cov_ + measurement_noise_;

  // Outlier rejection based on mahalanobis distance
  if(usbl_outlier_rejection_){
    mahalanobis_distance_ = innovation_vector_ / innovation_matrix_ * innovation_vector_;
    ROS_WARN_STREAM("ROTATIONAL: " << mahalanobis_distance_);
    if (mahalanobis_distance_ > outlier_threshold_) {
      ROS_WARN_STREAM("Docking Rotational: Measurement rejected as outlier (Mahalanobis distance = " << mahalanobis_distance_ << ")");
      outlier_rejected_ = 1;
      return false; 
    }
  }

  K_ = state_cov_ / innovation_matrix_;
  state_vec_ = wrapToPi(state_vec_ + K_ * innovation_vector_);
  state_cov_  = (1.0 - K_) * state_cov_;
  return true;
}
