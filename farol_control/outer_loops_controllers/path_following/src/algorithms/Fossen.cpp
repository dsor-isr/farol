#include "Fossen.h"

Fossen::Fossen(ros::Publisher surge_pub, ros::Publisher yaw_pub, ros::ServiceClient mode_client) :
  surge_pub_(surge_pub),
  yaw_pub_(yaw_pub),
  mode_client_(mode_client) {
  pfollowing_debug_.algorithm = "Fossen"; 
  }
/* Method to setup the gains of the controller */
bool Fossen::setPFGains(std::vector<double> gains) {
    int num_gains_received = gains.size();
  if(num_gains_received != this->num_gains_) return false;

  /* Asssign the gains */
  Delta_h = gains[0];

  return true;
  /* Fossen algorithm has no gains, therefore just return false */
  return false;
}

/**
 * @brief  Method that implements the path Following algorihtm 
 *
 * @param dt  The time difference in seconds
 */
void Fossen::callPFController(double dt) {
 
  /* Get the path paramerters */
  Eigen::Vector2d path_pd;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];
  double path_psi = this->path_state_.psi;
  double path_vd = this->path_state_.vd;
  double path_hg = this->path_state_.tangent_norm;

  /* Get the vehicle parameters */
  Eigen::Vector2d veh_p;
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];


  Eigen::Matrix2d RI_F;
  RI_F << cos(path_psi), sin(path_psi), -sin(path_psi), cos(path_psi);

  Eigen::Vector2d pos_error = RI_F * (veh_p - path_pd); 
  double y1 = pos_error[1];
  double ud;
  ud = path_hg * (path_vd + path_state_.vc);

  /* Save the control law to be published */
  this->desired_surge_ = std::min(std::max(ud, 0.0), 1.5);
  this->desired_yaw_ = path_psi + atan(-y1 / Delta_h);
  
  /* Convert the yaw from rad to deg (used by the inner-loop controller) */
  this->desired_yaw_ = this->desired_yaw_ * 180.0 / M_PI;

  /* Path following values for debug */
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = vehicle_state_.eta2[2];
  pfollowing_debug_.psi = path_state_.psi;
  pfollowing_debug_.gamma = path_state_.gamma;
}

/* Method to publish the control data */
void Fossen::publish_private() {
 
  /* Publish the control references */
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->yaw_pub_, this->desired_yaw_);
}

/* Method that will run in the first iteration of the algorithm */
void Fossen::start() {
  
  /* Fossen does not used this start needs to use the closest point to the path
   * and NOT the default gamma <-> data communication, therefore we should
   * inform the path to be used in closest point mode */
  dsor_paths::SetMode srv;
  srv.request.closest_point_mode = true;
  this->mode_client_.call(srv);  
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Fossen::stop() {
  
  /* If we have made all the path, then stop! */
  if(this->path_state_.gamma >= this->path_state_.gamma_max) {
    return true;
  }
  
  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Fossen::reset() {
  
  /* Reset the desired speed and yaw references */
  this->desired_surge_ = 0.0;
  this->desired_yaw_ = 0.0;

  return true;
}

