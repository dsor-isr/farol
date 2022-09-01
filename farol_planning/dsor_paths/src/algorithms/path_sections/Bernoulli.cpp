#include "Bernoulli.h"
#include <math.h>
#include <stdexcept>

Bernoulli::Bernoulli(double radius, double center_x, double center_y, double z) : PathSection(false) {
    /* Validate the parameters */
    if(radius <= 0) {
      throw std::invalid_argument("Bernoulli radius <= 0");
    }

    /* Assign the parameters */
    this->radius_ = radius;
    this->center_x_ = center_x;
    this->center_y_ = center_y;
    this->z_axis_ = z;
}

/* Constructors for the code*/
Bernoulli::Bernoulli(double radius, double center_x, double center_y) : Bernoulli(radius, center_x, center_y, 0.0) {}

/* Compute the path section equation */
Eigen::Vector3d Bernoulli::eq_pd(double t){
  
    Eigen::Vector3d pd_t(0.0, 0.0, 0.0);

    /* Compute the Bernoulli equation */
    double z = 1 + sin(t) * sin(t);
    pd_t[0] = this->radius_ * cos(t) / z + this->center_x_;
    pd_t[1] = this->radius_ * sin(t) * cos(t) / z + this->center_y_;
    pd_t[2] = this->z_axis_;

    return pd_t;
}

/* Compute the derivative of the path section */
Eigen::Vector3d Bernoulli::eq_d_pd(double t) {
  
    Eigen::Vector3d d_pd_t;
    
    double z = 1 + sin(t) * sin(t);

    /* Compute the derivatives */
    d_pd_t[0] = (this->radius_ * sin(t) * (sin(t) * sin(t) - 3)) / (z * z);
    d_pd_t[1] =
        -(this->radius_ * (3 * sin(t) * sin(t) - 1)) / pow((sin(t) * sin(t) + 1), 2);

    /* This is a 2D path in 2D plane */
    d_pd_t[2] = 0.0; 

    return d_pd_t;
}

/* Compute the second derivative of the path section */
Eigen::Vector3d Bernoulli::eq_dd_pd(double t) {

  //TODO: Compute the second derivative correctly
  Eigen::Vector3d dd_pd_t(0.0, 0.0, 0.0);

  return dd_pd_t;
}

/*Compute the curvature using the direct formula */
double Bernoulli::curvature(double t) {
  
  double num = 3 * sqrt(2) * cos(t);
  double den = this->radius_ * sqrt(3 - cos(2 * t));
  return num / den; 
}

/**
 * Method to return the closest point to the path 
 * By default just calls the Gradient Descent algorithm 
 */
double Bernoulli::getClosestPointGamma(Eigen::Vector3d &coordinate) {

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

