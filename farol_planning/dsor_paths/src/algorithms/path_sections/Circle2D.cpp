#include "Circle2D.h"
#include <stdexcept>

Circle2D::Circle2D(double radius, double center_x, double center_y, double z) : PathSection(false) {
 
  /* Validate the parameters */
  if(radius <= 0) {
    throw std::invalid_argument("Circle radius <= 0");
  }

  /* Save the parameters */
  this->radius_ = radius;
  this->center_x_ = center_x;
  this->center_y_ = center_y;
  this->z_axis_ = z;
}


Circle2D::Circle2D(double radius, double center_x, double center_y) : Circle2D(radius, center_x, center_y, 0.0) {}


Eigen::Vector3d Circle2D::eq_pd(double t) {
    
  Eigen::Vector3d pd_t;

  pd_t[0] = this->radius_ * cos(t) + this->center_x_;
  pd_t[1] = this->radius_ * sin(t) + this->center_y_;
  pd_t[2] = this->z_axis_;

  return pd_t;
}

Eigen::Vector3d Circle2D::eq_d_pd(double t) {
  
  Eigen::Vector3d d_pd_t;

  d_pd_t[0] = -this->radius_ * sin(t);
  d_pd_t[1] = this->radius_ * cos(t);
  d_pd_t[2] = 0.0;

  return d_pd_t;
}

Eigen::Vector3d Circle2D::eq_dd_pd(double t) {
  
  Eigen::Vector3d dd_pd_t;

  dd_pd_t[0] = -this->radius_ * cos(t);
  dd_pd_t[1] = -this->radius_ * sin(t);
  dd_pd_t[2] = 0.0;

  return dd_pd_t;
}

/* Compute the curvature only using the radius */
double Circle2D::curvature(double t) {
  return 1.0/this->radius_;
}

/**
 * Method to return the closest point to the path 
 * By default just calls the Gradient Descent algorithm 
 */
double Circle2D::getClosestPointGamma(Eigen::Vector3d &coordinate) {

  /* If it is the first iteration, try to get a good initialization, using bissection method */
  if(this->first_iteration_) {
    this->first_iteration_ = false;
    
    /* Search for the initial gamma in 4 different partions where in total
     * gamma varies between 0 and 2PI (as the bernoulli is periodic) */
    this->gamma_o_ = getInitialGammaEstimate(coordinate, this->num_partitions_, 0, 2 * M_PI);
    return this->gamma_o_;
  }

  double gamma_estimated = GradientDescent(this->gamma_o_, coordinate, this->tolerance_);

  /* Save this value to use as initialization for a next usage */
  this->gamma_o_ = gamma_estimated;
  return gamma_estimated;
}

