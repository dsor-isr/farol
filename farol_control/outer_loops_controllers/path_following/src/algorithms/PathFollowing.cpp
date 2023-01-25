#include "PathFollowing.h"

/* Virtual destructor for the path following class */
PathFollowing::~PathFollowing() {}

/* Setter for the vehicle state structure */
void PathFollowing::UpdateVehicleState(const VehicleState &vehicle_state) {
  this->vehicle_state_ = vehicle_state;
}

/* Setter for the path state structure */
void PathFollowing::UpdatePathState(const PathState &path_state) {
  this->path_state_ = path_state;
}

void PathFollowing::publish() {
  
  publish_private();

  farol_msgs::mPFollowingDebug pf_debug;

  pf_debug.header.stamp = ros::Time::now();
  pf_debug.algorithm = pfollowing_debug_.algorithm;
  pf_debug.cross_track_error = pfollowing_debug_.cross_track_error;
  pf_debug.along_track_error = pfollowing_debug_.along_track_error;
  pf_debug.yaw = pfollowing_debug_.yaw;
  pf_debug.psi = pfollowing_debug_.psi;
  pf_debug.gamma = pfollowing_debug_.gamma;
 
  pfollowing_debug_pub_.publish(pf_debug);
}

/* Auxiliar method to smooth out the angle to be used by path following algorithms */
double PathFollowing::algConvert(double alg_new, double alg_old, double alg_out_old) {
  double alg_e, alg_out;
  alg_e = alg_new - alg_old;
  if (alg_e > 3 * FarolGimmicks::PI / 2) {
    alg_out = alg_out_old - 2 * FarolGimmicks::PI + alg_e;
  } else if (alg_e < -3 * FarolGimmicks::PI / 2) {
    alg_out = alg_out_old + 2 * FarolGimmicks::PI + alg_e;
  } else {
    alg_out = alg_out_old + alg_e;
  }
  return alg_out;
}

double PathFollowing::preventPathSectionSwitching(double gamma, double gamma_dot, double dt) {
  // if gamma_dot is positive, then gamma will not go back to a previous section
  if (gamma_dot >= 0) return gamma_dot;

  // check if new gamma goes back
  double new_gamma = gamma + gamma_dot * dt;
  if (floor(new_gamma) != floor(gamma)) return 0.0;
  
  // else, return original gamma_dot
  return gamma_dot;
}

/* Method to reset the virtual target of the vehicle (gamma) to a pre-specified value. */
bool PathFollowing::resetVirtualTarget(float value) {

  /* By default do nothing, unless the controller implements this method */
  return true;
}

/* Method to reset the virtual target of the vehicle (gamma) to zero */
bool PathFollowing::resetVirtualTarget() {
  return this->resetVirtualTarget(0.0);
}