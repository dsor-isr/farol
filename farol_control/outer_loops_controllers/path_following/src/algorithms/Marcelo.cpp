#include "Marcelo.h"

/* The constructor for the Marcelo Path Following Controller */
Marcelo::Marcelo(double delta, double kk[2], double kz, double k_pos, double k_currents, double rd[3], double d[3], ros::Publisher surge_pub, ros::Publisher yaw_rate_pub, ros::Publisher rabbit_pub, ros::Publisher currents_estimation_x_pub, ros::Publisher currents_estimation_y_pub) {

  /* Initiate the gains of the controller */
  this->delta_ << 1.0, 0.0, 0.0, -delta;
  this->delta_inv_ << 1.0, 0.0, 0.0, -1/delta;
  this->kk_ << kk[0], 0.0, 0.0, kk[1];
  this->kz_ = kz;
  this->epsilon_ << delta, 0.0;

  /* Observer gains for the ocean currents */
  this->k_pos_ = k_pos;
  this->k_currents_ = k_currents;

  /* Save the offset for the path following */
  this->rd_ << rd[0], rd[1], rd[2];
  this->d_ << d[0], d[1], d[2];

  /* Save the publishers, to latter publish the data */
  this->surge_pub_ = surge_pub;
  this->yaw_rate_pub_ = yaw_rate_pub;
  this->rabbit_pub_ = rabbit_pub;
  this->currents_estimation_x_pub_ = currents_estimation_x_pub;
  this->currents_estimation_y_pub_ = currents_estimation_y_pub;
}

/* Method used to set the path following gains */
bool Marcelo::setPFGains(std::vector<double> gains) {

  /* Handle the case where the number of gains received does not coincide witht the
   * number of gains of the algorithm
   */
  if(gains.size() != 6) {
    return false;
  }
  
  /* Initiate the gains of the controller */
  this->delta_ << 1.0, 0.0, 0.0, -gains[0];
  this->delta_inv_ << 1.0, 0.0, 0.0, -1 / gains[0];
  this->kk_ << gains[1], 0.0, 0.0, gains[2];
  this->kz_ = gains[3];
  this->epsilon_ << gains[0], 0.0;

  /* Observer gains for the ocean currents */
  this->k_pos_ = gains[4];
  this->k_currents_ = gains[5];

  return true; //gains where updated successfully
}

/* Method that implements the control law */
void Marcelo::callPFController(double dt) {

  Eigen::Vector3d r1;
  Eigen::Vector3d r3;
  Eigen::Vector3d r2;
  r1 = this->path_state_.d_pd / this->path_state_.d_pd.norm();
  r3 = this->rd_ - (this->rd_.dot(r1) * r1);
  r3 = r3 / r3.norm();
  r2 = r3.cross(r1);

  Eigen::Matrix3d R_trans;
  for(int i = 0; i < 3; i++) {
      R_trans(i, 0) = r1(i);
      R_trans(i, 1) = r2(i);
      R_trans(i, 2) = r3(i);
  }

  /* Compute the actual desired position for the vehicle */
  Eigen::Vector3d pd;
  pd = this->path_state_.pd + (R_trans * this->d_);

  /* Get the vehicle position and orientation in the 2D plane */   
  Eigen::Vector2d vehicle_pos;
  Eigen::Vector2d vehicle_v1;
  Eigen::Vector2d path_pos;
  Eigen::Vector2d pd_dot;
  Eigen::Vector2d sway_vel;
  sway_vel << 0.0, 0.0;

  /* Get the path tangent to the norm */
  double path_hg = this->path_state_.tangent_norm; 
  path_pos << pd[0], pd[1];
  pd_dot << this->path_state_.d_pd[0], this->path_state_.d_pd[1];

  /* Get information from the vehicle */
  double vehicle_yaw = this->vehicle_state_.eta2[2];
  vehicle_pos << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];
  vehicle_v1 << this->vehicle_state_.v1[0], this->vehicle_state_.v1[1];

  /* Compute the rotation matrix from the inertial frame to the body frame */
  Eigen::Matrix2d rot_body;
  Eigen::Matrix2d rot_inert;
  rot_body << cos(vehicle_yaw), sin(vehicle_yaw), -sin(vehicle_yaw), cos(vehicle_yaw);
  rot_inert << cos(vehicle_yaw), -sin(vehicle_yaw), sin(vehicle_yaw), cos(vehicle_yaw);

  /* Estimator for the ocean currents */

  if(this->first_iteration_) {
    /* Initialize the complementary filter here */
    this->pos_hat_[0] = this->vehicle_state_.eta1[0];
    this->pos_hat_[1] = this->vehicle_state_.eta1[1];

    this->currents_hat_[0] = 0.0;
    this->currents_hat_[1] = 0.0;
    this->first_iteration_ = false;
  } else {
    /* Update the complementary filter used to estimate ocean currents */
    Eigen::Vector2d pos_hat_dot;
    Eigen::Vector2d currents_hat_dot;

    pos_hat_dot = this->k_pos_ * (vehicle_pos - this->pos_hat_) + (rot_inert * vehicle_v1) + this->currents_hat_;
    currents_hat_dot = this->k_currents_ * (vehicle_pos - this->pos_hat_);

    this->currents_hat_ += currents_hat_dot * dt;
    this->pos_hat_ += pos_hat_dot * dt;
  }

  /* Compute the ocean currents in the body frame */
  Eigen::Vector2d vcurrent;
  vcurrent = rot_body * this->currents_hat_;
 
  /* Compute the position error in the inertial frame */
  Eigen::Vector2d pos_error = rot_body * (vehicle_pos - path_pos) - this->epsilon_;
  Eigen::Vector2d pos_error_tanh;
  pos_error_tanh << tanh(pos_error[0]), tanh(pos_error[1]);

  /* PF Control law */
  Eigen::Vector2d vel_yaw_desired = this->delta_inv_ * (-this->kk_ * pos_error_tanh - sway_vel - vcurrent + rot_body * pd_dot * (this->path_state_.vd + this->path_state_.vc));

  /* Save the control values to be published later */
  this->desired_surge_ = vel_yaw_desired[0];

  /* NOTE: must convert this from rad/s to deg/s */
  this->desired_yaw_rate_ = vel_yaw_desired[1] * 180.0 / M_PI;

  /* Compute the error between the virtual target speed and its desired speed */
  double gamma_dot_error = this->gamma_dot_ - this->path_state_.vd - this->path_state_.vc;

  /* Virtual Target Control Law */
  double aux = pos_error.transpose() * rot_body * pd_dot;
  this->gamma_ddot_ = -this->kz_ * gamma_dot_error + (aux / path_hg)  + (this->path_state_.d_vd * this->gamma_dot_);
 
  /* Saturate the values of gamma to guarantee that if we start the */
  if (this->gamma_ <= this->path_state_.gamma_min && this->gamma_ddot_ < 0) {
    this->gamma_ddot_ = 0.0;
    this->gamma_dot_ = 0.0;
    this->gamma_ = this->path_state_.gamma_min;
  }
  
  // Integrate to get the virtual target position
  this->gamma_dot_ += this->gamma_ddot_ * dt;
  this->gamma_ += this->gamma_dot_ * dt;
 
  /* Path following values for debug */
  pfollowing_debug_.algorithm = "Marcelo";
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = vehicle_state_.eta2[2];
  pfollowing_debug_.psi = path_state_.psi;
  pfollowing_debug_.gamma = gamma_;
}

/* Method to publish_private the data computed from the algorithm */
void Marcelo::publish_private(){
    
  /* Publish the control references */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->yaw_rate_pub_, this->desired_yaw_rate_);

  /* Publish the virtual targets value */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);

  /* Publish the values of the current */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->currents_estimation_x_pub_, this->currents_hat_[0]);
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->currents_estimation_y_pub_, this->currents_hat_[1]);
}

/* Method that will run in the first iteration of the algorithm */
void Marcelo::start() {
  
  /* Publish the initial virtual target value to get the data from the path */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Marcelo::stop() {

  /**
   * Check if the gamma is greater then the gamma max of the path 
   * If so, we have reached the end
   */
  if(this->gamma_ >= this->path_state_.gamma_max) return true;

  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Marcelo::reset() {
  
  /* Reset the velocity references */
  this->desired_surge_ = 0.0;
  this->desired_yaw_rate_ = 0.0;

  /* Reset the value that holds the estimated position (used for currents estimation) */
  this->pos_hat_[0] = 0.0;
  this->pos_hat_[1] = 0.0;
  this->currents_hat_[0] = 0.0;
  this->currents_hat_[1] = 0.0;
  this->first_iteration_ = true;

  /* Reset the virtual target */
  this->gamma_ddot_ = 0.0;
  this->gamma_dot_ = 0.0;
  this->gamma_ = 0.0;

  return true;
}

/* Method to reset the virtual target of the vehicle (gamma) */
bool Marcelo::resetVirtualTarget(float value) {
  
  /* Reset the virtual target */
  this->gamma_ddot_ = 0.0;
  this->gamma_dot_ = 0.0;
  this->gamma_ = value;

  return true;
}