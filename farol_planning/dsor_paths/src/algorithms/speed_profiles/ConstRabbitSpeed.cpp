#include "ConstRabbitSpeed.h"

/* Constructor for the ConstRabbitSpeed class */
ConstRabbitSpeed::ConstRabbitSpeed(double rabbit_speed, double default_val) : 
  rabbit_speed_(rabbit_speed),
  default_speed_(default_val) {}

/* Method to get the desired velocity in the path frame */
double ConstRabbitSpeed::getVd(double gamma, double tangent_norm) {

  /* The desired rabbit speed is already in the path frame therefore no coversion is needed */
  return this->rabbit_speed_;
}

/* Method to get the derivative of the desired velocity in the path frame */
double ConstRabbitSpeed::get_d_Vd(double gamma, double tangent_norm) {

  /* The velocity desired is constant - we desire no acceleration */
  return 0.0;
}

/* Method to use as a default value if something goes wrong */
double ConstRabbitSpeed::getDefaultVd(double gamma, double tangent_norm) {
  
  /* The default in this case will be the same constant velocity */
  return this->default_speed_;
}
