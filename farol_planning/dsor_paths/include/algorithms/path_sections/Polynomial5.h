#pragma once

#include "PathSection.h"
#include <array>


/** 
 *  @brief     Class that implements a 5th order polynomial section
 *  @details   This class is used as a part of the sections library
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class Polynomial5 : public PathSection {

  public:
    /**
     * @brief  Constructor for the Polynomial5 class, that receives the desired plane for the polynomial
     *
     * @param a   A vector with 6 elements for the polynomial equation
     * @param b   A vector with 6 elements for the polynomial equation
     * @param c   A double for the polynomial equation
     * @param offset_x  A double with the x_offset
     * @param offset_y  A double with the y_offset
     */
    Polynomial5(Eigen::Matrix<double, 6, 1> &a, Eigen::Matrix<double, 6, 1> &b, double c, double offset_x, double offset_y);

    /**
     * @brief  The Path section equation 
     *
     * @param t  The path parameter
     *
     * @return  An Eigen::Vector3d with the equation of the path
     * with respect to the path parameter 
     */ 
    Eigen::Vector3d eq_pd(double t) override;

    /**
     * @brief First derivative of the path section equation with respect to the path parameter t 
     *
     * @param t  The path parameter t
     *
     * @return An Eigen::Vector3d with the equation of the partial derivative with respect to 
     * the path parameter 
     */
    Eigen::Vector3d eq_d_pd(double t) override;

    /** 
     * @brief  Second derivative of the path section equation with respect to the path parameter t 
     *
     * @param t  The path parameter t
     *
     * @return  An Eigen::Vector3d with the equation of the second order partial derivative with respect
     * to the path parameter
     */
    Eigen::Vector3d eq_dd_pd(double t) override;

    /**
     * @brief  Method for getting the gamma of the closes point to the path
     * 
     * @param   The coordinate of the vehicle in the 3D space
     *
     * @return  A double with the gamma corresponding to the closest point in the path 
     */
    double getClosestPointGamma(Eigen::Vector3d &coordinate) override;

  private:
    
    /**
     * @brief The parameters for the polynomial
     */
    Eigen::Matrix<double, 6, 1> a_;
    Eigen::Matrix<double, 6, 1> b_;
    double c_;
 
    /**
     * @brief Offsets to define where to place the curve
     */
    double offset_x_;
    double offset_y_;
};

