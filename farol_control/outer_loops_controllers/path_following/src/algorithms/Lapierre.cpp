#include "Lapierre.h"
#include <ros/ros.h> // TODO - remove - only used for debugging

/* Constructor for the Lapierre class */
Lapierre::Lapierre(double k1, double k2, double k3, double theta, double k_delta, ros::Publisher surge_pub, ros::Publisher yaw_rate_pub, ros::Publisher rabbit_pub) : 
  surge_pub_(surge_pub), 
  yaw_rate_pub_(yaw_rate_pub),
  rabbit_pub_(rabbit_pub) {
  
  /* Save the gains - no checkup is done here */
  this->k1_ = k1;
  this->k2_ = k2;
  this->k3_ = k3;
  this->theta_ = theta;
  this->k_delta_ = k_delta;
}

/* Method to setup the gains of the controller */
bool Lapierre::setPFGains(std::vector<double> gains) {

  /* Handle the special case where the number of parameters is not correct*/
  int num_gains_received = gains.size();
  if(num_gains_received != this->num_gains_) return false;

  /* Asssign the gains */
  this->k1_ = gains[0];
  this->k2_ = gains[1];
  this->k3_ = gains[2];
  this->theta_ = gains[3];
  this->k_delta_ = gains[4];

  return true;
}

/* The actual path following code */
void Lapierre::callPFController(double dt) { 

  /* Update the angles for the path and the vehicle in a continuous manner */
  this->smoothVehicleYaw();
  this->smoothPathYaw();

  /* Get the path data */
  Eigen::Vector2d path_pd;
  double path_psi = this->path_state_.psi;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];
  
  /* Get the path curvature and tangent to the norm */
  double path_cg = this->path_state_.curvature;
  double path_hg = this->path_state_.tangent_norm; 

  /* Get the vehicle data */
  Eigen::Vector2d veh_p; 
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1]; 

  /* Compute the Rotation matrix to the frenet frame */
  Eigen::Matrix2d RI_F;
  RI_F << cos(path_psi), sin(path_psi), -sin(path_psi), cos(path_psi);  

  /* Compute the errors s1 and y1 geometrically */
  Eigen::Vector2d pos_error = RI_F * (veh_p - path_pd);
 
  double s1 = pos_error[0];
  double y1 = pos_error[1];
  double psie = this->yaw_out_ - this->psi_out_;
 
  /* Compute the control law */
  this->desired_surge_ = path_hg * (this->path_state_.vd + this->path_state_.vc);

  /* Set the desired speed for the vehicle taking into account both vd and vc */
  double ud = path_hg * (this->path_state_.vd + this->path_state_.vc);
  double delta = -this->theta_ * tanh(this->k_delta_ * y1);
  double ydot = ud * sin(psie) - path_hg * path_cg * this->gamma_dot_ * s1;
  double delta_dot = -this->theta_ * this->k_delta_ * (1 - pow(tanh(this->k_delta_ * y1), 2)) * ydot;
  double psi_tilde = psie - delta;

  /* Controller */
  double uP = ud * cos(psie) + this->k3_ * s1; 
  double v_gamma = uP / path_hg;

  double r;  
  if (psi_tilde == 0) {
    r = delta_dot - this->k2_ * y1 * ud + path_cg * uP;
  } else {
    r = path_cg * uP + delta_dot - this->k1_ * psi_tilde - this->k2_ * y1 * ud * (sin(psie) - sin(delta)) / psi_tilde;
  } 

  /* Save the control values to be published */
  this->desired_surge_ = std::min(std::max(ud, 0.0), 1.5);
  this->desired_yaw_rate_ = r;
  
  /* Convert the yaw rate from radians/s to deg/s */
  this->desired_yaw_rate_ = this->desired_yaw_rate_ * 180.0 / M_PI;
  
  /* Control law for the virtual target 
   * NOTE: we must devide by the norm of the derivative of the path in order
   * to have the values expressed in the path frame and not in the inertial frame (the path 
   * might not be parameterized in distance
   */
  this->gamma_dot_ = v_gamma;

  /* Integrate to get the current velocity of the virtual target */
  this->gamma_ += this->gamma_dot_ * dt;

  /* Saturate the values of gamma, so that it does not achieve values for which we have no path */
  if (this->gamma_ < this->path_state_.gamma_min) this->gamma_ = this->path_state_.gamma_min;

  /* Path following values for debug */
  pfollowing_debug_.algorithm = "Lapierre";
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = yaw_out_;
  pfollowing_debug_.psi = psi_out_;
  pfollowing_debug_.gamma = gamma_;
}

/* Auxiliar method to smooth the vehicle angle */
void Lapierre::smoothVehicleYaw() {

  if (!isnan(this->vehicle_state_.eta2[2])) {
    this->yaw_out_ = this->algConvert(this->vehicle_state_.eta2[2], this->yaw_old_, this->yaw_out_old_);  
    this->yaw_old_ = this->vehicle_state_.eta2[2];
    this->yaw_out_old_ = this->yaw_out_;
  }
}

/* Auxiliar method to smooth the path angle */
void Lapierre::smoothPathYaw() {

  if (!isnan(this->path_state_.psi)) {
    this->psi_out_ = this->algConvert(this->path_state_.psi, this->psi_old_, this->psi_out_old_);
    this->psi_old_ = this->path_state_.psi;
    this->psi_out_old_ = this->psi_out_;
  }
}

/* Method to publish the data */
void Lapierre::publish_private() {
  
  /* Publish the control references */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->yaw_rate_pub_, this->desired_yaw_rate_);

  /* Publish the virtual targets value */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);

}

/* Method that will run in the first iteration of the algorithm */
void Lapierre::start() {

  /* Publish the initial virtual target to get the data from the path */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Lapierre::stop() {

  /**
   * Check if the gamma is greater then the gamma max of the path 
   * If so, we have reached the end
   */
  if(this->gamma_ >= this->path_state_.gamma_max) return true;

  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Lapierre::reset() {
  
  /* Reset the velocity references */
  this->desired_surge_ = 0.0;
  this->desired_yaw_rate_ = 0.0;
  
  /* Reset the auxiliary angles */
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
bool Lapierre::resetVirtualTarget(float value) {
  
  /* Reset the virtual target */
  this->gamma_dot_ = 0.0;
  this->gamma_ = value;

  return true;
}