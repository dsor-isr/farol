#include "BezierRabbitSpeed.h"

/* Constructor for the BezierRabbitSpeed class */
BezierRabbitSpeed::BezierRabbitSpeed(const Eigen::VectorXd &px, const  Eigen::VectorXd &py, const double Tf) {
  
  this->d_x_ = bezier_derivative(px)/Tf;
  this->d_y_ = bezier_derivative(py)/Tf;
  this->dd_x_ = bezier_derivative(this->d_x_)/Tf;
  this->dd_y_ = bezier_derivative(this->d_y_)/Tf;
}

/* Method to get the desired velocity in the path frame */
double BezierRabbitSpeed::getVd(double gamma, double tangent_norm) {

  int n1 = this->d_x_.size();
  int n2 = this->d_y_.size();

  double dx_gamma = deCasteljau(gamma, this->d_x_, n1, n1);
  double dy_gamma = deCasteljau(gamma, this->d_y_, n2, n2);

  double velocity = std::sqrt(dx_gamma*dx_gamma+dy_gamma*dy_gamma);
  ROS_INFO("I rabbit speed for gamma = %lf, the surge velocity is = %lf", gamma, velocity);
  
  return std::sqrt(dx_gamma*dx_gamma+dy_gamma*dy_gamma);
}

/* Method to get the derivative of the desired velocity in the path frame */
double BezierRabbitSpeed::get_d_Vd(double gamma, double tangent_norm) {

  int n1 = this->d_x_.size();
  int n2 = this->d_y_.size();

  double dx_gamma = deCasteljau(gamma, this->d_x_, n1, n1);
  double dy_gamma = deCasteljau(gamma, this->d_y_, n2, n2);

  double v_gamma = std::sqrt(dx_gamma*dx_gamma+dy_gamma*dy_gamma);

  n1 = this->dd_x_.size();
  n2 = this->dd_y_.size();

  double ddx_gamma = deCasteljau(gamma, this->dd_x_, n1, n1);
  double ddy_gamma = deCasteljau(gamma, this->dd_y_, n2, n2);

  return (ddx_gamma*dx_gamma+ddy_gamma*dy_gamma)/v_gamma;
}

/* Method to use as a default value if something goes wrong */
double BezierRabbitSpeed::getDefaultVd(double gamma, double tangent_norm) {
  
  int n1 = this->d_x_.size();
  int n2 = this->d_y_.size();

  double dx_gamma = deCasteljau(0, this->d_x_, n1, n1);
  double dy_gamma = deCasteljau(0, this->d_y_, n2, n2);

  return std::sqrt(dx_gamma*dx_gamma+dy_gamma*dy_gamma);
}

/* Compute control points of the derivative of a Bézier Curve */
Eigen::VectorXd BezierRabbitSpeed::bezier_derivative(const Eigen::VectorXd &P)
{
    int n = P.size();
    int degree = n - 1;

    Eigen::VectorXd P_derivative(degree);
    for (int i = 0; i < degree; ++i)
    {
        P_derivative(i) = degree * (P(i + 1) - P(i));
    }
    return P_derivative;
}


/* De Casteljau's algorithm to get specific points on the curve*/
double BezierRabbitSpeed::deCasteljau(double u, const Eigen::VectorXd &pts, int i, int j)
{
    if (i == 1)
    {
        return pts(j - 1);
    }
    else
    {
        double left = deCasteljau(u, pts, i - 1, j);
        double right = deCasteljau(u, pts, i - 1, j - 1);
        return u * left + (1 - u) * right;
    }
}
