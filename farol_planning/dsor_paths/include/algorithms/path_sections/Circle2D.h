#pragma once

#include "PathSection.h"

/** 
 *  @brief     Class that implements a 2D Circle section
 *  @details   This class is used as a part of the sections library
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class Circle2D : public PathSection {

  public:

    /** 
     * @brief Constructor for the path section that assumes the plane
     * in which the path is placed is z=0
     *
     * @param radius  The radius in m of the Circle
     * @param center_x The x coordinate of the center of the circle
     * @param center_y The y coordinate of the center of the circle
     */
    Circle2D(double radius, double center_x, double center_y);

    /** 
     * @brief Constructor for the path section 
     *
     * @param radius  The radius in m of the Circle
     * @param center_x The x coordinate of the center of the circle
     * @param center_y The y coordinate of the center of the circle
     * @param z The z axis where to place the circle
     */
    Circle2D(double radius, double center_x, double center_y, double z);

    /** 
     * @brief Path Section equation 
     *
     * @param t  The path parameter
     *
     * @return  An Eigen::Vector3d with the values from the equation of the path 
     */
    Eigen::Vector3d eq_pd(double t) override;

    /**
     * @brief First derivative of the path section equation with respect to the path parameter 
     *
     * @param t  The path parameter
     *
     * @return  An Eigen::Vector3d with the values from the derivative of the equation 
     */
    Eigen::Vector3d eq_d_pd(double t) override;

    /**
     * @brief  Second derivative of the path section equation with respect to the path parameter t 
     *
     * @param t  The path parameter
     *
     * @return  An Eigen::Vector3d with the values from the second derivative of the equation
     */
    Eigen::Vector3d eq_dd_pd(double t) override;

    /**
     * @brief  Compute the curvature using only the radius. Computed as 1/Radius
     *
     * @param t  The path paramter
     *
     * @return  A double with the curvature of that section (1/radius) 
     */
    double curvature(double t) override;

    /** 
     * @brief  Method for getting the gamma of the closest point 
     *
     * @param coordinate  The vehicle position
     *
     * @return  A double with the gamma corresponding to the closest point in the path
     */
    double getClosestPointGamma(Eigen::Vector3d &coordinate) override;

  private:

    /**
     * @brief The radius of the circle path 
     */
    double radius_;

    /**
     * @brief The center coordinates for the Circle
     */
    double center_x_, center_y_;

    /** 
     * @brief The desired 2D plane in which to place the circle 
     */
    double z_axis_;

    /**
     * @brief Auxiliary variable to do a better optimization when searching for the closest point 
     */
    bool first_iteration_{true};
    double gamma_o_{0.0};
    const int num_partitions_{3};
    const double tolerance_{0.01};
};

