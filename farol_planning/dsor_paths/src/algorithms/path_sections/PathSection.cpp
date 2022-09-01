#include "PathSection.h"
#include <math.h>
#include <limits>
#include <stdexcept>

/* Constructor for the Path section */
PathSection::PathSection(bool can_be_composed) : can_be_composed_(can_be_composed) {

  /* By default, if can_be_composed is true, the section must vary between [0 - some value] */
  if(this->can_be_composed_ == true) this->min_value_gamma_ = 0.0;
}

/* Virtual Destructor for the path section */
PathSection::~PathSection() {}

/* Default method for computing the tangent to a path section */
double PathSection::tangent(double t) {

  /* Limit the path parameter between the minimum and maximum value */
  t = this->limitGamma(t);

  /* Compute deivative of the path section */
  Eigen::Vector3d d_pd = this->eq_d_pd(t);

  /* Compute the tangent to the 2D path */
  double psi = atan2(d_pd[1], d_pd[0]);

  return psi;
}

/* Default method for computing the path curvature */
double PathSection::curvature(double t) {

  /* Limit the path parameter between the minimum and maximum value */
  t = this->limitGamma(t);

  Eigen::Vector3d d_pd = this->eq_d_pd(t);
  Eigen::Vector3d dd_pd = this->eq_dd_pd(t);

  double curvature = (d_pd[0] * dd_pd[1] - d_pd[1] * dd_pd[0]) / (pow(d_pd.norm(), 3));

  return curvature;
}

/* Computes the norm of the derivative of the path section */
double PathSection::derivative_norm(double t) {

  /* Limit the path parameter between the minimum and maximum value */
  t = this->limitGamma(t);

  return this->eq_d_pd(t).norm();
}

/* Returns whether the path section can be composed with other sections or not */
bool PathSection::can_be_composed() {
  return this->can_be_composed_;
}

/* Method to update the max value of the gamma parameter */
bool PathSection::setMaxGammaValue(double gamma_max) {

  /* Only update if the gamma_max is greater than gamma_min */
  if (gamma_max > this->min_value_gamma_) {
    this->max_value_gamma_ = gamma_max;
    return true;
  }

  /* Otherwise do not update the value as its not valid */
  return false;
}

/* Method to update the min value of the gamma paramter */
bool PathSection::setMinGammaValue(double gamma_min) {

  /* Only consider update if gamma_min is smaller than gamma_max*/
  if (gamma_min < this->max_value_gamma_) {

    /* If this section cannot be compose with other, any value will do */
    if(this->can_be_composed_ == false) {
      this->min_value_gamma_ = gamma_min;
      return true;

      /* If this section can be composed with other sections, it will not update the minimum
       * value as its required for the minimum value to be zero */
    } else {
      return false;
    }
  }

  /* Otherwise do not update that value as its not valid */
  return false;
}

/* Method to get the max value the gamma can achieve */
double PathSection::getMaxGammaValue() {
  return this->max_value_gamma_;
}

/* Method to get the min value the gamma can achieve */
double PathSection::getMinGammaValue() {
  return this->min_value_gamma_;
}


/* Auxiliar method to limit gamma between 0 and the max value of gamma */
double PathSection::limitGamma(double t) {

  if(t > this->max_value_gamma_) t = this->max_value_gamma_;
  if(t < this->min_value_gamma_) t = this->min_value_gamma_;

  return t;
}

/**
 * Method to return the closest point to the path 
 * By default just calls the Gradient Descent algorithm 
 */
double PathSection::getClosestPointGamma(Eigen::Vector3d &coordinate) {

  /* If it is the first iteration, try to get a good initialization, using bissection method */
  if(this->first_iteration_) {
    this->first_iteration_ = false;
    this->gamma_o_ = getInitialGammaEstimate(coordinate, this->default_num_partitions_, this->min_value_gamma_, this->max_value_gamma_);
    return this->gamma_o_;
  }

  double gamma_estimated = GradientDescent(this->gamma_o_, coordinate, this->tolerance_);

  /* Save this value to use as initialization for a next usage */
  this->gamma_o_ = gamma_estimated;
  return gamma_estimated;
}

/* Method to compute the gradient descent */
double PathSection::GradientDescent(double gamma_o, Eigen::Vector3d &x_pos, double tolerance) {

  /* Stop flag */
  bool stop = false;

  /* The initial estimate value for the parameter gamma */
  double gamma = gamma_o;
  double g_k = 0.0;
  double d_k = 0.0;
  double alpha_k = 0.0;

  /* Check the stoping criteria */
  while (stop == false) {

    /* Compute the gradient of the function */
    g_k = this->grad_F(gamma, x_pos);

    /* Check the stopping criteria */
    if (std::abs(g_k) > tolerance) {

      /* Set the descent direction the oposite of the gradient */
      d_k = -g_k;

      /* Do the backtracking to have adpatative step size */
      alpha_k = this->backtrack(gamma, x_pos, d_k, g_k, this->alpha_hat_, this->beta_, this->epsilon_);

      /* Update theta_k+1 = theta_k + alpha_k*d_k */
      gamma = gamma + alpha_k * d_k;

      /* (Addition to the algorithm) If gamma is outside the bounds of */
      if(gamma < this->getMinGammaValue()) {
        stop = true;
        return this->getMinGammaValue();

      } else if (gamma > this->getMaxGammaValue()) {
        stop = true;
        return this->getMaxGammaValue();
      }

    } else {  
      stop = true;
    }
  }

  return gamma;
}

/* Method to compute the derivative of the function we are trying to minimize */
double PathSection::grad_F(double gamma, Eigen::Vector3d &x_pos) {

  Eigen::Vector3d pd = this->eq_pd(gamma);
  Eigen::Vector3d d_pd = this->eq_d_pd(gamma);

  return (pd - x_pos).transpose() * d_pd;
}

/* Method to compute the function we are trying to minimize */
double PathSection::F(double gamma, Eigen::Vector3d &x_pos) {

  Eigen::Vector3d pd = this->eq_pd(gamma);
  double diff_norm = (pd - x_pos).norm();

  return diff_norm;
}

/* 
 * Method to compute the backtrack for the gradient descent in order to have
 * adaptative step size 
 */
double PathSection::backtrack(double gamma, Eigen::Vector3d &x_pos, double d_k, double grad_f, double alpha_hat, double beta, double epsilon) {

  /* Auxiliar variables for the computations */
  double f1;
  double f2;
  double f3;

  /* Stop flag */
  bool stop = false;

  /* Auxiliary variable to check if we are stuck */
  double last_f1 = std::numeric_limits<double>::max();

  /* The initial value for the parameter alpha */
  double alpha_k = alpha_hat;

  while(stop == false) {

    /* Compute the f(gamma_k + alpha_k * d_k) */
    f1 = this->F(gamma + alpha_k * d_k, x_pos);

    /* Compute the f(gamma_k) */
    f2 = this->F(gamma, x_pos);

    /* Compute the gradient of f(xk)' * (alpha_k * d_k)*/
    f3 = epsilon * grad_f * (alpha_k * d_k);

    /* (Addition to the algorithm) Check if we are stuck */
    if (last_f1 == f1) {
      /* Since our function is finite, we must guarantee we do not get stuck */
      return alpha_k;
    } else {
      last_f1 = f1;
    }

    /* Check if we should update the alpha */
    if(f1 > f2 + f3) {
      alpha_k = beta * alpha_k;
    } else {
      stop = true;
    }
  }
  return alpha_k;
}

/* Auxiliar method to get the initial gamma estimate when using the 
 * path in closest point mode
 */
double PathSection::getInitialGammaEstimate(Eigen::Vector3d &x_pos, int num_partitions, double min_val, double max_val) {

  /* Validate the arguments */
  if(num_partitions <= 0 || min_val > max_val) {
    throw std::invalid_argument("Arguments of getInitialGammaEstimate method are invalid");
  }

  /* Divide the section into n parts and search for local minimums
   * in those n parts of the section. The minimum of those local minimus
   * have higher change of being the global minimum 
   */
  double l = (max_val - min_val) / num_partitions;
  
  /* Compute the most likely gamma for the minimum of each partition */
  double begin_value = min_val;
  double error = std::numeric_limits<double>::max();
  double error_best = std::numeric_limits<double>::max();
  double gamma = 0.0;
  double gamma_best = 0.0;

  for(int i = 0; i < num_partitions; i++) {

    /* Compute the best gamma for this partition */
    gamma = bisection(x_pos, begin_value, begin_value + l);
    
    /* Compute the distance when using that gamma */
    error = (x_pos - this->eq_pd(gamma)).norm();
   
    /* If the distance error is smaller than the best until now, update the best */
    if (error < error_best) {
      error_best = error;
      gamma_best = gamma;
    }
    
    /* Increment the partition to look at */
    begin_value += l;
  }

  return gamma_best;
}

/* Compute the bissection of a section */
double PathSection::bisection(Eigen::Vector3d &x_pos, double a, double b) {

  double c = a;
 
  /* Iterate on the bissection method until the threshold is met */
  while ((b - a) >= this->EP_bissection_) {

    /* Find middle point */
    c = (a + b) / 2;

    /* Check if middle point is root */
    if (grad_F(c, x_pos) == 0.0)
      break;
    /* Decide the side to repeat the steps */
    else if (grad_F(c, x_pos) * grad_F(a, x_pos) < 0)
      b = c;
    else
      a = c;
  }
  return c;
}


