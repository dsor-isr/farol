#include "ConstVehicleSpeed.h"
#include <cmath>

/* Constructor for the ConstVehicleSpeed class */
ConstVehicleSpeed::ConstVehicleSpeed(double vehicle_speed, double default_val) :
  vehicle_speed_(vehicle_speed),
  default_speed_(default_val) {}

/* Method to get the desired velocity in the path frame */
double ConstVehicleSpeed::getVd(double gamma, double tangent_norm) {

  double gamma_speed = 0.0;
  
  /* Convert the speed from the vehicle frame to the path frame */
  if(tangent_norm != 0) gamma_speed = this->vehicle_speed_ / tangent_norm;

  /* If the tangent_norm was 0, then gamma_speed is infinite! Limit this! */
  if(tangent_norm == 0 || !(std::isfinite(gamma_speed))) gamma_speed = 1.0;

  return gamma_speed;
}

/* Method to get the derivative of the desired velocity in the path frame */
double ConstVehicleSpeed::get_d_Vd(double gamma, double tangent_norm) {

  /* The velocity desired is constant - we desire no acceleration */
  return 0.0;
}

/* Method to use as a default value if something goes wrong */
double ConstVehicleSpeed::getDefaultVd(double gamma, double tangent_norm) {
  
  /* The default in this case will be the same constant velocity */
  return this->default_speed_ / tangent_norm;
}
