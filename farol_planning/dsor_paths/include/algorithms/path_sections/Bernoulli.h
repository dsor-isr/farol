#pragma once

#include "PathSection.h"

/** 
 *  @brief     Class that implements a 2D Lemniscate of Bernoulli section
 *  @details   This class is used as a part of the sections library
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class Bernoulli : public PathSection {

  public:

    /**
     * @brief Constructor for the path section that assumes
     * the plane in which the path is placed is z=0 
     *
     * @param radius   The radius of the bernoulli lamniscaste section
     * @param center_x The x coordinate of the center of the bernoulli
     * @param center_y The y coordinate of the center of the bernoulli
     */
    Bernoulli(double radius, double center_x, double center_y);

    /**
     * @brief Constructor for the path section 
     *
     * @param radius   The radius of the bernoulli lamniscaste section
     * @param center_x The x coordinate of the center of the bernoulli
     * @param center_y The y coordinate of the center of the bernoulli
     * @param z        The altitude in m in which to place the bernoulli
     */
    Bernoulli(double radius, double center_x, double center_y, double z);

    /**
     * @brief Path section equation
     *
     * @param t The path parameter
     * 
     * @return  An Eigen::Vector3d with the value of the equation of the path 
     */
    Eigen::Vector3d eq_pd(double t) override;

    /**
     * @brief  First derivative of the path section equation with respect to the path parameter t 
     *
     * @param t  The path parameter
     *
     * @return  An Eigen::Vector3d with the value of the derivate of the equation of the path 
     */
    Eigen::Vector3d eq_d_pd(double t) override;

    /**
     * @brief  Second derivative of the path section equation with respect to the path parameter t 
     *
     * @param t  The path paramter
     *
     * @return  An Eigen::Vector3d with the value of the derivative of the equation of the path
     */
    Eigen::Vector3d eq_dd_pd(double t) override;

    /**
     * @brief  Compute the curvature using the direct formula 
     *
     * @param t  The path parameter
     *
     * @return a double with the curvature of the section 
     */
    double curvature(double t) override;

    /**
     * @brief  Method for getting the gamma of the closest point. In this implementation
     * the closest point is computed iteratively using Gradient Descent Method. For the first
     * iteration, to get a good initialization, the section is divided into N partitions between
     * [0, 2*PI] and the minimum is computed for each partition. The gamma with the minimum
     * distance will be used to initialize the algorithm.
     *
     * @param coordinate  An Eigen::Vector3d with the coordinate of the vehicle
     *
     * @return a double with the gamma corresponding to the closest point
     */
    double getClosestPointGamma(Eigen::Vector3d &coordinate) override;

  private:

    /** 
     * @brief The radius of the bernoulli path 
     */
    double radius_;

    /**
     * @brief The x, y coordinates of the center of the bernoulli path 
     */
    double center_x_, center_y_;

    /** 
     * @brief  The desired 2D plane in which to place the Bernoulli 
     */
    double z_axis_;

    /**
     * @brief Auxiliary variable to do a better optimization when searching for the closest point 
     */
    bool first_iteration_{true};
    double gamma_o_{0.0};
    const int num_partitions_{4};
    const double tolerance_{0.01};
};

