#include "Samson.h"

/* Constructor for the Lapierre class */
Samson::Samson(double k1, double k2, double k3, double theta, double k_delta, ros::Publisher surge_pub, ros::Publisher yaw_rate_pub, ros::ServiceClient mode_client) : 
  surge_pub_(surge_pub), 
  yaw_rate_pub_(yaw_rate_pub),
  mode_client_(mode_client) {
  
  /* Save the gains - no checkup is done here */
  this->k1_ = k1;
  this->k2_ = k2;
  this->k3_ = k3;
  this->theta_ = theta;
  this->k_delta_ = k_delta;
  pfollowing_debug_.algorithm = "Samson"; 
}

/* Method to setup the gains of the controller */
bool Samson::setPFGains(std::vector<double> gains) {

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
void Samson::callPFController(double dt) { 

  /* Update the angles for the path and the vehicle in a continuous manner */
  this->smoothVehicleYaw();
  this->smoothPathYaw();

  /* Get the path data */
  Eigen::Vector2d path_pd;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];

  /* Get the path curvature and tangent to the norm */
  double path_cg = this->path_state_.curvature;
  double path_hg = this->path_state_.tangent_norm;
  double path_vd = this->path_state_.vd;

  /* Get the vehicle data */
  Eigen::Vector2d veh_p; 
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1]; 

  /* Compute the rotation matrix */
  Eigen::Matrix2d RI_F;
  RI_F << cos(this->psi_out_), sin(this->psi_out_), -sin(this->psi_out_), cos(this->psi_out_);

  Eigen::Vector2d pos_error = RI_F * (veh_p - path_pd);

  /* Compute the errors */ 
  double y1 = pos_error[1];
  double psie = this->yaw_out_ - this->psi_out_;

  double ud, r;
  ud = path_hg * (path_vd + path_state_.vc);
  double delta = -this->theta_ * tanh(this->k_delta_ * y1);
  double ydot = ud * sin(psie);
  double delta_dot = -this->theta_ * this->k_delta_ * (1 - pow(tanh(this->k_delta_ * y1), 2)) * ydot;
  double psi_tilde = psie - delta;

  /* Compute the control law */
  double uP = ud * cos(psie) / (1 - path_cg * y1);

  if (psie == delta) {
    r = delta_dot - (this->k2_ * y1 * ud) + (path_cg * uP);
  } else {
    r = (path_cg * uP) + delta_dot - (this->k1_ * psi_tilde) - this->k2_ * y1 * ud * (sin(psie) - sin(delta)) / psi_tilde;
  }

  /* Saturate the desired surge speed */
  this->desired_surge_ = std::min(std::max(ud, 0.0), 1.5);
  this->desired_yaw_rate_ = r;
  
  /* Convert the yaw_rate from rad/s to deg/s (used by the inner-loop controller) */
  this->desired_yaw_rate_ = this->desired_yaw_rate_ * 180.0 / M_PI;

  /* Path following values for debug */
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = yaw_out_;
  pfollowing_debug_.psi = psi_out_;
  pfollowing_debug_.gamma = path_state_.gamma;

}

/* Auxiliar method to smooth the vehicle angle */
void Samson::smoothVehicleYaw() {

  if (!isnan(this->vehicle_state_.eta2[2])) {
    this->yaw_out_ = this->algConvert(this->vehicle_state_.eta2[2], this->yaw_old_, this->yaw_out_old_);  
    this->yaw_old_ = this->vehicle_state_.eta2[2];
    this->yaw_out_old_ = this->yaw_out_;
  }
}

/* Auxiliar method to smooth the path angle */
void Samson::smoothPathYaw() {

  if (!isnan(this->path_state_.psi)) {
    this->psi_out_ = this->algConvert(this->path_state_.psi, this->psi_old_, this->psi_out_old_);
    this->psi_old_ = this->path_state_.psi;
    this->psi_out_old_ = this->psi_out_;
  }
}


/* Method to publish the data */
void Samson::publish_private() {

  /* Publish the control references */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->yaw_rate_pub_, this->desired_yaw_rate_); 
}

/* Method that will run in the first iteration of the algorithm */
void Samson::start() {
  /* Samson does not used this start needs to use the closest point to the path
   * and NOT the default gamma <-> data communication, therefore we should
   * inform the path to be used in closest point mode */
  dsor_paths::SetMode srv;
  srv.request.closest_point_mode = true;
  this->mode_client_.call(srv); 
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Samson::stop() {

  /* If we have made all the path, then stop! */
  if(this->path_state_.gamma >= this->path_state_.gamma_max) {
    return true;
  }

  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Samson::reset() {

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

  return true;
}


