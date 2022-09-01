#include "Brevik.h"

/* The constructor for the Aguiar Path Following Controller */
Brevik::Brevik(ros::Publisher surge_pub, ros::Publisher yaw_pub, ros::Publisher rabbit_pub) {

  /* Save the publishers, to latter publish the data */
  this->surge_pub_ = surge_pub;
  this->yaw_pub_ = yaw_pub;
  this->rabbit_pub_ = rabbit_pub;
}

/* Method used to set the path following gains */
bool Brevik::setPFGains(std::vector<double> gains) {
  // Breviks algorithm receives no gains
  return false; 
}

/* Method that implements the control law */
void Brevik::callPFController(double dt) {
  
  /* Update the angles for the path and the vehicle in a continuous manner */
  this->smoothVehicleYaw();
  this->smoothPathYaw();
  
  /* Get the path parameters */
  Eigen::Vector2d path_pd;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];
  double path_hg = this->path_state_.tangent_norm;
  double path_vd = this->path_state_.vd;

  /* Get the vehicle parameters */
  Eigen::Vector2d veh_p;
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];

  double Delta_h = 2;

  /* Compute the rotation matrix */
  Eigen::Matrix2d RI_F;
  RI_F << cos(this->psi_out_), sin(this->psi_out_), -sin(this->psi_out_), cos(this->psi_out_);

  /* Compute the error */
  Eigen::Vector2d pos_error = RI_F * (veh_p - path_pd);
  double s1 = pos_error[0];
  double y1 = pos_error[1];
  double psie = this->yaw_out_ - this->psi_out_;

  /* Compute the control law */
  double ud;
  ud = path_hg * (path_vd + path_state_.vc);

  /* Save the control law to be published */
  this->desired_surge_ = std::min(std::max(ud, 0.0), 1.5);
  this->desired_yaw_ = this->path_state_.psi + atan(-y1 / Delta_h);

  /* Convert the yaw from rad to deg (used by the inner-loop controller) */
  this->desired_yaw_ = this->desired_yaw_ * 180.0 / M_PI;

  /* Compute the evolution of the virtual target */
  double k3 = 1.0;
  double uP = ud * cos(psie) + k3 * s1;
  
  double v_gamma = uP / path_hg;
  this->gamma_dot_ = v_gamma;
  this->gamma_dot_ = std::min(std::max(this->gamma_dot_, -0.5), 0.5);

  /* Saturate the values of gamma to guarantee that if we start the */
  if (this->gamma_ <= this->path_state_.gamma_min && this->gamma_dot_ < 0) {
    this->gamma_dot_ = 0.0;
    this->gamma_ = this->path_state_.gamma_min;
  }
  
  /* Integrate to get the virtual target position */
  this->gamma_ += this->gamma_dot_ * dt;
  
  /* Path following values for debug */
  pfollowing_debug_.algorithm = "Brevik"; 
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = yaw_out_;
  pfollowing_debug_.psi = psi_out_;
  pfollowing_debug_.gamma = gamma_;
}

/* Auxiliar method to smooth the vehicle angle */
void Brevik::smoothVehicleYaw() {
  
  if (!isnan(this->vehicle_state_.eta2[2])) {
    this->yaw_out_ = this->algConvert(this->vehicle_state_.eta2[2], this->yaw_old_, this->yaw_out_old_);  
    this->yaw_old_ = this->vehicle_state_.eta2[2];
    this->yaw_out_old_ = this->yaw_out_;
  }
}

/* Auxiliar method to smooth the path angle */
void Brevik::smoothPathYaw() {
  
  if (!isnan(this->path_state_.psi)) {
    this->psi_out_ = this->algConvert(this->path_state_.psi, this->psi_old_, this->psi_out_old_);
    this->psi_old_ = this->path_state_.psi;
    this->psi_out_old_ = this->psi_out_;
  }
}

/* Method to publish the data computed from the algorithm */
void Brevik::publish_private(){
    
  /* Publish the control references */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->yaw_pub_, this->desired_yaw_);

  /* Publish the virtual targets value */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method that will run in the first iteration of the algorithm */
void Brevik::start() {
  
  /* Publish the initial virtual target value to get the data from the path */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Brevik::stop() {

  /**
   * Check if the gamma is greater then the gamma max of the path 
   * If so, we have reached the end
   */
  if(this->gamma_ >= this->path_state_.gamma_max) return true;

  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Brevik::reset() {
  
  /* Reset the velocity references */
  this->desired_surge_ = 0.0;
  this->desired_yaw_ = 0.0;
  
  /* Reset the auxiliar variables for the angles*/
  this->yaw_out_ = 0.0;
  this->yaw_out_old_ = 0.0;
  this->yaw_old_ = 0.0;
    
  this->psi_out_ = 0.0;
  this->psi_out_old_ = 0.0;
  this->psi_old_ = 0.0;

  /* Reset the virtual target */
  this->gamma_dot_ = 0.0;
  this->gamma_ = 0.0;

  return true;
}

/* Method to reset the virtual target of the vehicle (gamma) */
bool Brevik::resetVirtualTarget(float value) {
  
  /* Reset the virtual target */
  this->gamma_dot_ = 0.0;
  this->gamma_ = value;

  return true;
}