#pragma once

#include "PathSection.h"


/** 
 *  @brief     Class that implements a 2D arc section
 *  @details   This class is used as a part of the sections library
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class Arc2D : public PathSection {

  public:

    /**
     * @brief  Constructor for the Arc2D class, that receives the desired plane for the arc 
     *
     * @param start_point   The 2D start point of the arc
     * @param end_point     The 2D end point of the arc
     * @param center_point  The 2D coordinate with the center of the arc
     * @param direction     An int (-1 or 1) that represents the direction of the arc 
     * @param z             The altitude at which to place the arc 
     */
    Arc2D(Eigen::Vector2d start_point, Eigen::Vector2d end_point, Eigen::Vector2d center_point, int direction, double z);

    /**
     * @brief  Constructor for the Arc2D class, that receives the desired plane for the arc.
     * Assumes the plane is placed at z=0.0 m
     *
     * @param start_point   The 2D start point of the arc
     * @param end_point     The 2D end point of the arc
     * @param center_point  The 2D coordinate with the center of the arc
     * @param direction     An int (-1 or 1) that represents the direction of the arc 
     */
    Arc2D(Eigen::Vector2d start_point, Eigen::Vector2d end_point, Eigen::Vector2d center_point, int direction);


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
     * @brief  Compute the curvature in a simple manner. Computed as 1/Radius 
     *
     * @param t   The path parameter t
     *
     * @return  A double with the curvature in of that section (1/radius)
     */
    double curvature(double t) override;

    /**
     * @brief  Method for getting the gamma of the closes point to the path in a more efficient manner 
     * For an arc it uses a closed form equation to solve this problem
     * 
     * @param coordinate The coordinate of the vehicle in the 3D space
     *
     * @return  A double with the gamma corresponding to the closest point in the path 
     */
    double getClosestPointGamma(Eigen::Vector3d &coordinate) override;

  private:

    /**
     * @brief The arc start point
     */
    Eigen::Vector2d start_point_;

    /**
     * @brief The arc end point
     */
    Eigen::Vector2d end_point_;

    /**
     * @brief The arc center point
     */
    Eigen::Vector2d center_point_;

    /**
     * @brief The direction to use 
     */
    int direction_; 

    /** 
     * @brief The desired plane in which to place the arc 
     */
    double z_axis_;

    /** 
     * @brief Auxiliar parameters for computations 
     * The Radius of the curve
     */
    double R_;

    /**
     * @brief Auxiliar parameters for the computations
     * The initial angle
     */
    double psi0_;

    /**
     * @brief Auxiliar parameters for the computations
     * The final angle
     */
    double psif_;
};

