#pragma once

#include "PathSection.h"


/** 
 *  @brief     Class that implements a 5th order polinomial section
 *  @details   This class is used as a part of the sections library
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class Polinomial5 : public PathSection {

  public:

    /**
     * @brief  Constructor for the Polinomial5 class, that receives the desired plane for the polinomial
     *
     * @param offset        The 2D start point of the arc 
     */
    Polinomial5(std::array<double, 5> a, std::array<double, 5> b, double c);

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
     * @brief The offset for the sinudoidal path
     */
    Eigen::Vector2d offset_;
    
    /** 
     * @brief The desired plane in which to place the arc 
     */
    double z_axis_;
};

