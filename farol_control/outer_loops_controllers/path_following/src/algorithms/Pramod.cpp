#include "Pramod.h"

Pramod::Pramod(std::vector<double> gains, ros::Publisher surge_pub, ros::Publisher yaw_pub, ros::ServiceClient mode_client) :
  surge_pub_(surge_pub),
  yaw_pub_(yaw_pub), 
  mode_client_(mode_client) {
  /* NOTE: no gain checkup is performed here */
  this->setPFGains(gains);
}

/* Method to setup the gains of the controller */
bool Pramod::setPFGains(std::vector<double> gains) {
  
  /* Handle the case where the number of gains received is not correct */
  if(gains.size() != 2) return false;

  this->gains_ = gains;
  return true;
}

/* saturation as described in paper*/
double sigma_e(double input){
   if (input > 1)
    return 1;
  else if (input < -1)
    return -1;
  return input;
}

void Pramod::callPFController(double dt) {
 
  /* Get the path paramerters */
  Eigen::Vector2d path_pd;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];
  double path_psi = this->path_state_.psi;
  double path_vd = this->path_state_.vd;
  double path_hg = this->path_state_.tangent_norm;

  /* Get the vehicle parameters */
  Eigen::Vector2d veh_p;
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];
  double veh_surge = this->vehicle_state_.v1[0];

  /* Compute the rotation matrix */ 
  Eigen::Matrix2d RI_F;
  RI_F << cos(path_psi), sin(path_psi), -sin(path_psi), cos(path_psi);
  
  /* Compute the position error */
  Eigen::Vector2d pos_error = RI_F * (veh_p - path_pd);
  static double cross_track = pos_error[1];
 
  /* Notation used is the described in "A Path-Following Controller for Marine Vehicles Using a Two-Scale Inner-Outer Loop Approach" by Pramod Maurya (https://www.mdpi.com/1424-8220/22/11/4293) 
     "cross_track" is the cross track error, represented by "e" in the paper
     "zeta" is the integral of the cross track error which has a dynamic of its own which is described id the paper 
     "sigma_e" is the saturation function described in the paper
     finally, each important line of code has above it the cooresponent equation from the paper
  */
  
  cross_track = pos_error[1];
  static double zeta = cross_track*dt;

  /*Anti windup gain*/
  double Ka = 1/dt; /* = 1/Ts */
  
  /* zeta dynamics with anti-windup */
  double zeta_dot = cross_track + Ka *( 
    -this->gains_[0] / veh_surge * cross_track - this->gains_[1] / veh_surge * zeta - 
    sigma_e( -this->gains_[0] / veh_surge * cross_track - this->gains_[1] / veh_surge * zeta)
    );

  /* integrate to obtain zeta */
  zeta += zeta_dot*dt; 

  /* u = -K1/U*e - K2/U*sigma */
  double yaw_correction = -this->gains_[0] / veh_surge * cross_track - this->gains_[1] / veh_surge * zeta;
  
  /* psi_d = path_psi + asin(sat(u)) */
  double desired_yaw_rad = path_psi + asin(sigma_e(yaw_correction));
  
  this->desired_yaw_ = desired_yaw_rad * 180 / M_PI;
  this->desired_surge_ = (path_vd + path_state_.vc) * path_hg;

  /* Path following values for debug */
  pfollowing_debug_.algorithm = "Pramod";
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = vehicle_state_.eta2[2];
  pfollowing_debug_.psi = path_state_.psi;
  pfollowing_debug_.gamma = path_state_.gamma;
}

/* Method to publish the control data */
void Pramod::publish_private() {
 
  /* Publish the control references */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->yaw_pub_, this->desired_yaw_);
}

/* Method that will run in the first iteration of the algorithm */
void Pramod::start() {
  /* Pramod does not used this start needs to use the closest point to the path
   * and NOT the default gamma <-> data communication, therefore we should
   * inform the path to be used in closest point mode */
  dsor_paths::SetMode srv;
  srv.request.closest_point_mode = true;
  this->mode_client_.call(srv); 
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Pramod::stop() {
  
  /* If we have made all the path, then stop! */
  if(this->path_state_.gamma >= this->path_state_.gamma_max) {
    return true;
  }
  
  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Pramod::reset() {
  
  /* Reset the desired speed and yaw references */
  desired_surge_ = 0.0;
  desired_yaw_ = 0.0;

  return true;
}

