#include "Line.h"
#include <stdexcept>

/* Constructor that assumes the line is given relative to a reference point */
Line::Line(Eigen::Vector3d start_point, Eigen::Vector3d end_point, Eigen::Vector3d ref_point) : PathSection(true) {
  
  /* Validate if the line is valid, otherwise just return insucess */
  if((start_point - end_point).norm() < 0.0000001) {
    throw std::invalid_argument("Line: Initial and end point are the same");
  }

  /* Instantiate the class attributes */
  this->start_point_ = start_point;
  this->end_point_ = end_point;
  this->ref_point_ = ref_point;

  /* Make sure the gamma varies between 0 and 1*/
  this->setMinGammaValue(0.0);
  this->setMaxGammaValue(1.0);
}

/* Constructor that assumes that the line has no reference point to it */
Line::Line(Eigen::Vector3d start_point, Eigen::Vector3d end_point) : Line(start_point, end_point, Eigen::Vector3d(0.0, 0.0, 0.0)) {}


/* Compute the equation of a line */
Eigen::Vector3d Line::eq_pd(double t) {

  Eigen::Vector3d slope;
  Eigen::Vector3d offset;
 
  /* Make sure the path parameter is between 0 and 1 */
  t = this->limitGamma(t);

  /* Compute the slope and offest of the line */
  slope = this->end_point_ - this->start_point_;
  offset = this->start_point_;
 
  return slope * t + offset;
}

/* Compute the derivative equation of line */
Eigen::Vector3d Line::eq_d_pd(double t) {

  /* Compute the slope of the line */
  return this->end_point_ - this->start_point_;
}

/* Compute the second derivative equation of line */ 
Eigen::Vector3d Line::eq_dd_pd(double t) {

  /* The second derivative of a line is 0 */
  Eigen::Vector3d dd_p(0.0, 0.0, 0.0);

  return dd_p;
}

/* Compute the curvature of a line */
double Line::curvature(double t) {
  return 0.0;
}

/* Method for getting the gamma of the closes point to the path in a more efficient manner */
double Line::getClosestPointGamma(Eigen::Vector3d &coordinate) {
       
    // Line Distance
    double l = (this->end_point_ - this->start_point_).norm();         
    
    // Distance to the begining of the mission
    double ds = (coordinate - this->start_point_).norm(); 
    
    // Distance to the end of the mission 
    double de = (coordinate - this->end_point_).norm();

    // Change this for any g0 and ge
    double gamma = this->getMinGammaValue() + ((l * l + ds * ds - de * de) / (2 * l));   
   
    // Normalize so that gamma varies between gamma min and gamma max (aka [0,1])
    gamma = gamma / l;
    
    // Saturate between the minimum and maximum values of gamma
    if(gamma > this->getMaxGammaValue()) {
      gamma = this->getMaxGammaValue();
    }

    if(gamma < this->getMinGammaValue()) {
      gamma = this->getMinGammaValue();
    }

    return gamma;
}

