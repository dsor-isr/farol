#include "Polynomial5.h"

Polynomial5::Polynomial5(Eigen::Matrix<double, 6, 1> &a, Eigen::Matrix<double, 6, 1> &b, double c, double offset_x, double offset_y) : PathSection(true) {
  
  /* Save the parameters for the curve */
  this->a_ = a;
  this->b_ = b;
  this->c_ = c;
  this->offset_x_ = offset_x;
  this->offset_y_ = offset_y;
}

Eigen::Vector3d Polynomial5::eq_pd(double t) {
 
  double z = t + this->c_;

  Eigen::Matrix<double, 6, 1> phi;
  phi << pow(z, 5), pow(z, 4), pow(z, 3), pow(z, 2), z, 1;
  
  Eigen::Vector3d pd; 
  pd << (this->a_.transpose() * phi) + this->offset_x_, (this->b_.transpose() * phi) + this->offset_y_, z;

  return pd;
}
Eigen::Vector3d Polynomial5::eq_d_pd(double t) {
  //TODO
  return Eigen::Vector3d(0.0, 0.0, 0.0); 
}

Eigen::Vector3d Polynomial5::eq_dd_pd(double t) {
  //TODO
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}

double Polynomial5::getClosestPointGamma(Eigen::Vector3d &coordinate) {
  //TODO
  return 0.0;
}

