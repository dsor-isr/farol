#pragma once

#include <Eigen/Core>
#include <math.h>
#include <limits>

/** 
 *  @brief     An abstract class that is used as a template for Path Sections
 *  @details   For path sections that can_be_composed the minimum value 
 *             of gamma will be defaulted to 0 (and cannot be changed) in 
 *             order not to mess with the Path section switching algorithm. 
 *             The maximum value can be any greater than 0.
 *
 *             For path sections where can_be_composed = false, the minimum 
 *             value for gamma if -inf an the maximum value if +inf 
 *             (but these can be changed)
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class PathSection {

  public:  

    /**
     * @brief  The Path section equation 
     *
     * @param t  The path parameter
     *
     * @return  An Eigen::Vector3d with the equation of the path
     * with respect to the path parameter 
     */ 
    virtual Eigen::Vector3d eq_pd(double t) = 0;

    /**
     * @brief  First derivative of the path section equation 
     * with respect to path parameter t 
     *
     * @param t  The path parameter
     *
     * @return  An Eigen::Vector3d with the first derivative of the path equation
     * with respect to the path parameter
     */
    virtual Eigen::Vector3d eq_d_pd(double t) = 0;

    /**
     * @brief  Second derivative of the path section equation with 
     * respect to the path parameter t 
     *
     * @param t  The path parameter
     *
     * @return  An Eigen::Vector3d with the second derivative of the path equation 
     * with respect to the path paramter
     */
    virtual Eigen::Vector3d eq_dd_pd(double t) = 0;

    /**
     * @brief  Default method for computing the tangent to the path section 
     *
     * @param t  The path parameter
     *
     * @return  A double with the angle of the tangent to the path 
     */
    virtual double tangent(double t);

    /** 
     * @brief  Default method for computing the curvature. The default implementation
     * implements the general formula to compute the curvature based on the derivative
     * equations of the path
     *
     * @param t  The path parameter
     *
     * @return  A double with the path curvature 
     */
    virtual double curvature(double t);

    /**
     * @brief  Default method for computing the norm of the derivative 
     *
     * @param t  The path parameter
     *
     * @return  A double with the norm of the derivative of the path position pd
     */
    virtual double derivative_norm(double t);

    /**
     * @brief  Default method for getting the gamma of the closed point to the path. 
     * By default this method uses gradient Descent algorithm to compute the closest
     * point in the path with the initial guess of gamma=0.0.
     *
     * This is not the most efficient way to compute the closest point for several kinds
     * of paths, but it is the most general, hence used as the default one
     *
     * @param coordinate  A Eigen::Vector3d with the coordinates of the vehicle
     *
     * @return  A double with the closest point in the path
     */
    virtual double getClosestPointGamma(Eigen::Vector3d &coordinate);

    /**
     * @brief  Method to return whether a pathSection can be composed with other
     * path sections or not
     *
     * @return  A boolean indicating wether the pathsection can be composed with 
     * other sections or not
     */
    bool can_be_composed();

    /**
     * @brief  Method to limit the gamma between the minimum and maximum value  
     * By default the maximum gamma is a very high number 
     *
     * @param t  The path parameter also known as gamma
     *
     * @return  A double with the path parameter limited between the valid bounds
     */
    double limitGamma(double t);

    /**
     * @brief  Method used to get the maximum gamma allowed by the path section
     * By the default is the maximum valid number possible in c++
     *
     * @return  A double with the maximum value that can be achieved with gamma
     */
    double getMaxGammaValue();

    /**
     * @brief Method used to get the minimum gamma allowed by the path section
     * By default is the minimum valid number possible in c++
     *
     * @return  A double with the minimum value the can be achieved with gamma
     */
    double getMinGammaValue();

    /**
     * @brief  Virtual destructor for the abstract class
     */
    virtual ~PathSection();

  protected:

    /**
     * @brief Constructor for the abstract class i
     * NOTE: this class is virtual, therefore an object of type PathSection
     * cannot be instantiated in memory. Only derivatives of the class PathSection
     *
     * @param can_be_composed  A boolean that indicated that if this kind of path
     * can be composed with other path sections or not
     *
     */
    PathSection(bool can_be_composed);

    /** 
     * @brief  Method to update the max of the gamma parameter 
     * Validates if the value received is greater than gamma_min
     *
     * @param gamma_max  The desired max value for gamma
     * 
     * @return  True if new value was accepted 
     */
    bool setMaxGammaValue(double gamma_max);

    /**
     * @brief  Method to update the min value of the gamma parameter
     * Validades if the value is received is smaller than gamma_max
     *
     * @param gamma_min The desired min value for gamma
     *
     * @return  True if the new value was accepted 
     *
     * NOTE: for paths where can_be_composed == true, this function
     * always returns false as it is required for those kinds of segments
     * to start with 0 (but no limit is put on gamma max)
     */
    bool setMinGammaValue(double gamma_min);

    /**
     * @brief  Method that implements the gradient descent to minimize the error
     * of ||pd(gamma) - p_vehicle||
     *
     * @param gamma_o    The initial guess for the path paramter
     * @param x_pos      The position of the vehicle
     * @param tolerance  A constant tweaking parameter
     *
     * @return the estimated gamma value that minimizes the error
     */
    double GradientDescent(double gamma_o, Eigen::Vector3d &x_pos, double tolerance);

    /**
     * @brief  Method to get an initial estimate for the gamma. It divides the section
     * into n sections and search for local minimums in the function that computes the distance
     * of the vehicle inside those sections.
     *
     * Then grabs the gamma that minimizes the most from all sections
     *
     * @param x_pos           The position of the vehicle
     * @param num_partitions  The number of divisions to make on the path
     * @param min_val         The minimum boundary to search in gamma
     * @param max_val         The maximum boundary to search in gamma
     *
     * @return A double with a good gamma to use as an initialization for the 
     * estimation
     */
    double getInitialGammaEstimate(Eigen::Vector3d &x_pos, int num_partitions, double min_val, double max_val);

    /**
     * @brief  Method to implement bisection method  
     *
     * @param x_pos The position of the vehicle
     * @param a     The left bound for the gamma value 
     * @param b     The right bound for the gamma value
     *
     * @return    The most likely value of gamma
     */
    double bisection(Eigen::Vector3d &x_pos, double a, double b);


  private:

    /**
     * @brief  The min and max value for the gamma for a particular path section 
     */
    double max_value_gamma_{std::numeric_limits<double>::max() / 2};
    double min_value_gamma_{std::numeric_limits<double>::lowest() / 2};

    /** 
     * @brief  Variable to check whether this section can be used in a composition of sections or not 
     */
    bool can_be_composed_{true};

    /**
     * @brief Auxiliar variables to use by the gradient descent 
     * algorithm when getting the closest point to the path by default
     */
    const double alpha_hat_{0.01};
    const double epsilon_{0.0001};
    const double beta_{0.5};
    const double tolerance_{0.1};
    double gamma_o_{0.0};
    bool first_iteration_{true};
    const int default_num_partitions_{3};

    /** 
     * @brief Auxiliar variables for bissection method 
     */
    const double EP_bissection_{0.001};

    /**
     * @brief  Method to implement backtrack for gradient descent (for adaptative step size)
     *
     * @param gamma     The path parameter
     * @param x_pos     The position of the vehicle
     * @param d_k       The descent direction
     * @param grad_f    The derivative of the function we are minimizing
     * @param alpha_hat A constant tweaking parameter
     * @param beta      A constant tweaking parameter
     * @param epsilon   A constant tweaking parameter
     *
     * @return the new value for alpha_k (the step size)
     */
    double backtrack(double gamma, Eigen::Vector3d &x_pos, double d_k, double grad_f, double alpha_hat, double beta, double epsilon); 

    /**
     * @brief  Method to compute the error between ||pd(gamma) - vehicle_pos||
     *
     * @param gamma     The path parameter
     * @param x_pos     The position of the vehicle
     *
     * @return A double with ||pd(gamma) - vehicle_pos||
     */
    double F(double gamma, Eigen::Vector3d &x_pos);

    /**
     * @brief  Method to compute the derivative of ||pd(gamma) - vehicle_pos||
     *
     * @param gamma     The path parameter
     * @param x_pos     The position of the vehicle
     * 
     * @return A double with the derivative of ||pd(gamma) - vehicle_pos||
     * with respect to gamma
     */
    double grad_F(double gamma, Eigen::Vector3d &x_pos); 
};

