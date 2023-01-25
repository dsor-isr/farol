#include "Romulo.h"

/* Constructor for the Romulo class */
Romulo::Romulo(std::vector<double> gains, ros::Publisher surge_pub, ros::Publisher sway_pub, ros::Publisher rabbit_pub) :  
  surge_pub_(surge_pub), 
  sway_pub_(sway_pub),
  rabbit_pub_(rabbit_pub) {

    /* Save the gains for the controller */
    /* NOTE: I'M ASSUMING THE PARAMETERS ARE VALID - NO CHECK IS DONE HERE */
    this->setPFGains(gains);
  }

/* Method to setup the path following gains */
bool Romulo::setPFGains(std::vector<double> gains) {

  /* Handle the special case where the number of gains does not coincide with the controller gains */
  if(gains.size() != 5) {
    return false;
  }

  /* Store the valid values of the gains */
  this->gains_ = gains;
  this->ke << this->gains_[0], this->gains_[1], this->gains_[2], this->gains_[3];
  this->kz = gains_[4];

  return true;
}

/* Method that implements the path following control law */
void Romulo::callPFController(double dt) {

  /* Get the vehicle yaw and position from the data */
  double veh_yaw = this->vehicle_state_.eta2[2];

  Eigen::Vector2d veh_p;
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];

  /* Get the path position */
  Eigen::Vector2d path_pd;
  Eigen::Vector2d path_pd_dot;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];
  path_pd_dot << this->path_state_.d_pd[0], this->path_state_.d_pd[1];
  double path_hg = this->path_state_.tangent_norm;

  /* Compute the rotation matrix */
  Eigen::Matrix2d rot_body;
  rot_body << cos(veh_yaw), sin(veh_yaw), -sin(veh_yaw), cos(veh_yaw);

  /* Compute the position error between the path and the vehicle */
  Eigen::Vector2d pos_error = rot_body * (path_pd - veh_p);

  /* Compute the control law for surge and sway */
  Eigen::Vector2d velocity_desired = rot_body * (this->path_state_.vd + this->path_state_.vc) * path_pd_dot + ke * pos_error;

  /* Update the surge and sway references */
  this->desired_surge_ = std::min(std::max(velocity_desired[0], -0.5), 0.5);
  this->desired_sway_ = std::min(std::max(velocity_desired[1], -0.5), 0.5);

  /* Compute the error for the virtual target */
  double z = (this->path_state_.vd + this->path_state_.vc) - this->gamma_dot_;

  /* Compute the control law for the virtual rabbit*/
  double aux = pos_error.transpose() * rot_body * path_pd_dot;
  this->gamma_ddot_ = this->gamma_dot_ * this->path_state_.d_vd - (aux / path_hg) + kz * z;

  // Integrate to get the virtual target velocity
  this->gamma_dot_ += this->gamma_ddot_ * dt;

  /* Make sure gamma does not return to a previous path section, given that
  each path section is paramaterised from 0 to 1 */
  this->gamma_dot_ = this->preventPathSectionSwitching(this->gamma_, this->gamma_dot_, dt);

  // Integrate to get the virtual target position
  this->gamma_ += this->gamma_dot_ * dt;

  /* Saturate the values of gamma */
  if (this->gamma_ < this->path_state_.gamma_min) this->gamma_ = this->path_state_.gamma_min;

  /* Path following values for debug */
  pfollowing_debug_.algorithm = "Romulo";
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = vehicle_state_.eta2[2];
  pfollowing_debug_.psi = path_state_.psi;
  pfollowing_debug_.gamma = gamma_;
}

/* Method to publish the data from the path following */
void Romulo::publish_private() {
  
  /* Publish the control references */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->sway_pub_, this->desired_sway_);

  /* Publish the virtual targets value */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method that will run in the first iteration of the algorithm */
void Romulo::start() {

  /* Publish in initial virtual target to get the values from the path */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Romulo::stop() {

  /**
   * Check if the gamma is greater then the gamma max of the path 
   * If so, we have reached the end
   */
  if(this->gamma_ >= this->path_state_.gamma_max) return true;

  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Romulo::reset() {
  
  /* Reset the velocity references */
  this->desired_surge_ = 0.0;
  this->desired_sway_ = 0.0;

  /* Reset the virtual target */
  this->gamma_ddot_ = 0.0;
  this->gamma_dot_ = 0.0;
  this->gamma_ = 0.0;

  return true;
}

/* Method to reset the virtual target of the vehicle (gamma) */
bool Romulo::resetVirtualTarget(float value) {
  
  /* Reset the virtual target */
  this->gamma_ddot_ = 0.0;
  this->gamma_dot_ = 0.0;
  this->gamma_ = value;

  return true;
}

