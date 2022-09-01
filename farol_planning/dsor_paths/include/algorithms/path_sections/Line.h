#pragma once

#include "PathSection.h"

/** 
 *  @brief     Class that implements a 3D line section
 *  @details   This class is used as a part of the sections library
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class Line : public PathSection {

  public:

    /**
     * @brief  Constructor for the 3DLine class. It assumes that the reference
     * for the frame is (0, 0, 0)
     *
     * @param start_point  The starting point in the 3D space
     * @param end_point    The end point in the 3D space
     */
    Line(Eigen::Vector3d start_point, Eigen::Vector3d end_point);
    
    /**
     * @brief  Constructor for the 3DLine class
     *
     * @param start_point  The starting point in the 3D space
     * @param end_point    The end point in the 3D space
     * @param ref_point    The ref_point in the 3D to draw the line relative to this reference
     */
    Line(Eigen::Vector3d start_point, Eigen::Vector3d end_point, Eigen::Vector3d ref_point);

    /**
     * @brief Path section equation
     *
     * @param t  The path parameter
     *
     * @return An Eigen::Vector3d with the value of the equation of the path 
     */
    Eigen::Vector3d eq_pd(double t) override;

    /** 
     * @brief First derivative of the path section equation with respect to the path parameter t 
     *
     * @param t  The path parameter
     *
     * @return An Eigen::Vector3d with the value of the derivative of the path equation 
     */
    Eigen::Vector3d eq_d_pd(double t) override;

    /**
     * @brief  Second derivative of the path section equation with respect to the path parameter t 
     *
     * @param t  The path parameter
     *
     * @return An Eigen::Vector3d with the value of the second derivative of the path equation
     */
    Eigen::Vector3d eq_dd_pd(double t) override;

    /** 
     * @brief  The curvature of a line is constant, therefore we can make the computation really efficient 
     *
     * @param t  The path parameter
     *
     * @return A double = 0 (since a line has no curvature)
     */
    double curvature(double t) override;

    /**
     * @brief  Method for getting the gamma of the closes point to the path in a more efficient manner 
     * For a line it uses a closed form equation to solve this problem
     *
     * @param coordinate  The vehicle coordinate
     *
     * @return A double with the gamma corresponding to the cloest point in the path.
     */
    double getClosestPointGamma(Eigen::Vector3d &coordinate) override;

  private:

    /**
     * @brief The start point of the line
     */
    Eigen::Vector3d start_point_;

    /**
     * @brief The end point of the line
     */
    Eigen::Vector3d end_point_;
    
    /**
     * @brief The reference to use to draw the line upon
     */
    Eigen::Vector3d ref_point_;    
};

