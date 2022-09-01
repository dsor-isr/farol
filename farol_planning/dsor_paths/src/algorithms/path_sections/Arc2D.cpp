#include "Arc2D.h"
#include <math.h>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <stdexcept>

/**
 * Constructor for the Arc2D path.
 * Using this constructor we can specify the plane in which to put the arc
 */
Arc2D::Arc2D(Eigen::Vector2d start_point, Eigen::Vector2d end_point, Eigen::Vector2d center_point, int direction, double z) : PathSection(true) {
  
  /* Validate if the arc parameters  */
  if(((start_point - center_point).norm() < 0.0000001) || 
     ((start_point - end_point).norm()    < 0.0000001) || 
     ((center_point - end_point).norm()   < 0.0000001)) {
    throw std::invalid_argument("Arc2D: Some of the arguments are equal");
  }

  /* Assign the parameters of the path */
  this->start_point_ = start_point;
  this->end_point_ = end_point;
  this->center_point_ = center_point;
  this->direction_ = direction;
  this->z_axis_ = z; 
  
  /* Compute the arc radius to be used later */
  this->R_ = (center_point - start_point).norm();

  /* Compute the initial angle */
  this->psi0_ = atan2(start_point[1] - center_point[1], 
                      start_point[0] - center_point[0]);

  /* Set the gamma max for this path to be between 0 and 1 */
  this->setMinGammaValue(0.0);
  this->setMaxGammaValue(1.0);
}

/**
 * The constructors for the Arc2D paths
 * Using this constructor we assume the path is place in the plane z = 0
 */
Arc2D::Arc2D(Eigen::Vector2d start_point, Eigen::Vector2d end_point, Eigen::Vector2d center_point, int direction) : Arc2D(start_point, end_point, center_point, direction, 0.0) {};


/* Path Section equation */
Eigen::Vector3d Arc2D::eq_pd(double t) {
  
  Eigen::Vector2d pd;
  Eigen::Vector3d pd3D;

  /* Make sure the path parameter is betwen 0 and 1*/
  t = this->limitGamma(t);

  /* Compute the angle */
  double aux_angle = this->psi0_ - this->direction_ * t * M_PI;

  /* Compute the desired position coordinates */ 
  pd = this->center_point_ + this->R_ * Eigen::Vector2d(cos(aux_angle), sin(aux_angle));
    
  /* Place the path in a 3D space */
  pd3D[0] = pd[0];
  pd3D[1] = pd[1];
  pd3D[2] = this->z_axis_;

  return pd3D;
}

/* First derivative of the path section */
Eigen::Vector3d Arc2D::eq_d_pd(double t) {

  Eigen::Vector2d d_pd;
  Eigen::Vector3d d_pd3D;

  /* Make sure the path parameter is betwen 0 and 1*/
  t = this->limitGamma(t);

  /* Compute the angle */
  double aux_angle = this->psi0_ - this->direction_ * t * M_PI;
  
  /* Compute the derivative of the desired position */
  d_pd = -this->direction_ * this->R_ * M_PI * Eigen::Vector2d(-sin(aux_angle), cos(aux_angle));

  /* Place the result in a 3D space */
  d_pd3D[0] = d_pd[0];
  d_pd3D[1] = d_pd[1];
  d_pd3D[2] = 0.0;

  return d_pd3D;
}

/* Second derivative of the path section */
Eigen::Vector3d Arc2D::eq_dd_pd(double t) {
  
  Eigen::Vector2d dd_pd;
  Eigen::Vector3d dd_pd3D;
  
  /* Make sure the path parameter is betwen 0 and 1*/
  t = this->limitGamma(t);

  /* Compute the angle */
  double aux_angle = this->psi0_ - this->direction_ * t * M_PI;

  /* Compute the second derivative */
  dd_pd = pow(this->direction_, 2) * pow(M_PI, 2) * this->R_ * Eigen::Vector2d(-cos(aux_angle), -sin(aux_angle));

  /* Place the result in a 3D space */
  dd_pd3D[0] = dd_pd[0];
  dd_pd3D[1] = dd_pd[1];
  dd_pd3D[2] = 0.0;

  return dd_pd3D;
}

/* Compute the curvature using only the radius */
double Arc2D::curvature(double t) {
   
  return (double) this->direction_ * 1.0 / this->R_;
}

/* Method for getting the gamma of the closes point to the path in a more efficient manner */
double Arc2D::getClosestPointGamma(Eigen::Vector3d &coordinate) {
    
    // The final angle
    double psie = atan2(this->end_point_[1] - this->center_point_[1],
                        this->end_point_[0] - this->center_point_[0]);
   
    // The current actual angle
    double psi = atan2(coordinate[1] - this->center_point_[1], 
                       coordinate[0] - this->center_point_[0]);
    
    double psi_init = FarolGimmicks::wrapTo2pi(this->psi0_);
    double psi_end = FarolGimmicks::wrapTo2pi(psie);
    double psi_act = FarolGimmicks::wrapTo2pi(psi);
      
    psi_end = FarolGimmicks::wrapTo2pi(psi_end - psi_init);
    psi_act = FarolGimmicks::wrapTo2pi(psi_act - psi_init);
    
    // For a normalized radius of 1
    double arc_len = fabs((this->direction_ == -1) ? 2 * M_PI - psi_end : psi_end);
    double arc_to_init = fabs((this->direction_ == -1) ? 2 * M_PI - psi_act : psi_act);

    // Closer to the beggining
    arc_to_init = -(2 * M_PI - arc_to_init);
        
    // Get the actual value of the gamma
    double aux = this->getMinGammaValue() + arc_to_init / arc_len * (this->getMaxGammaValue() - this->getMinGammaValue());
    aux = std::abs(aux);
  
    // Saturate the values between the minimum and maximum values
    if(aux < this->getMinGammaValue()) aux = this->getMinGammaValue();
    if(aux > this->getMaxGammaValue()) aux = this->getMaxGammaValue();
 
    return aux;
}

