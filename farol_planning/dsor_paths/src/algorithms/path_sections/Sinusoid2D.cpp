#include "Sinusoid2D.h"

Sinusoid2D::Sinusoid2D(Eigen::Vector2d &offset, double z) : PathSection(true) {
  //TODO
}

Sinusoid2D::Sinusoid2D(Eigen::Vector2d &offset) : Sinusoid2D(offset, 0.0) {}

Eigen::Vector3d Sinusoid2D::eq_pd(double t) {
  //TODO
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}
Eigen::Vector3d Sinusoid2D::eq_d_pd(double t) {
  //TODO
  return Eigen::Vector3d(0.0, 0.0, 0.0); 
}

Eigen::Vector3d Sinusoid2D::eq_dd_pd(double t) {
  //TODO
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}

double Sinusoid2D::getClosestPointGamma(Eigen::Vector3d &coordinate) {
  //TODO
  return 0.0;
}

