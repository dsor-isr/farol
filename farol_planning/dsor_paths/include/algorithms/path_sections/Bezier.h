#pragma once

#include "PathSection.h"

/** 
 *  @brief     Class that implements a 2D Bezier Parametrization of a Curve
 *  @details   This class is used as a part of the sections library
 *  @author    Gorka Monteiro
 *  @version   1.0a
 *  @date      2025
 *  @copyright MIT
 */
class Bezier : public PathSection {

  public:

    /**
     * @brief Constructor for the path section 
     *
     * @param Control_points
     */
    Bezier(const Eigen::VectorXd &px, const  Eigen::VectorXd &py, int bez_deg,double z, double Tf
    );

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
     * @brief  Method for getting the gamma of the closest point. In this implementation
     * the closest point is computed using the GJK algorithm
     *
     * @param coordinate  An Eigen::Vector3d with the coordinate of the vehicle
     *
     * @return a double with the gamma corresponding to the closest point
     */
    double getClosestPointGamma(Eigen::Vector3d &coordinate) override;

    /**
     * @brief  Recursive method for getting a specific point on a Bézier curve using the Casteljau's algorithm
     *
     * @param u value between 0 and 1 that represents gamma
     * @param pts Control points
     * @param i,j Auxiliary values for the recursive part
     * 
     * @return C(u) value
     */
    Eigen::Vector2d deCasteljau(double u, const Eigen::Matrix2Xd &pts, int i, int j);

     /**
     * @brief  Calculate the Control points of a Derivative 
     *
     * @param P Control points
     * 
     * @return Control points of the Derivative
     */
    Eigen::Matrix2Xd bezier_derivative(const Eigen::Matrix2Xd &P);

    /**
     * @brief  Divide Bezier into 2 new pieces
     *
     * @param P Control points
     * @param u Point at which divides
     * 
     * @return Control points of each part
     */
    std::pair<Eigen::Matrix2Xd, Eigen::Matrix2Xd> divide_bezier(const Eigen::Matrix2Xd &P, double u);

    /**
     * @brief  Implemant GJK algorthm
     *
     * @param P Control points of square of distance
     * @param alpha initialize big value (its where the min value gets stored)
     * @param epsilon initialize small (its the stop criterion)
     * @param u_start, u_min Aux values to return the final gamma values
     * 
     * @return Control points of each part
     */
    std::pair<double, double> GJK(const Eigen::Matrix2Xd &P, double alpha, double epsilon, double u_start, double u_end);

    /**
     * @brief  Implemant GJK algorthm
     *
     * @param P Control points of square of distance
     * @param alpha initialize big value (its where the min value gets stored)
     * @param epsilon initialize small (its the stop criterion)
     * @param u_start, u_min Aux values to return the final gamma values
     * 
     * @return Control points of each part
     */
    Eigen::Matrix2Xd multiply_Bezier(const Eigen::Matrix2Xd &P1, const Eigen::Matrix2Xd &P2);

    double Tf_;
    
  private:

    /** 
     * @brief The Control Points of the Bézier Curva
     */
    Eigen::Matrix2Xd Control_points_;

    /** 
     * @brief  The desired 2D plane in which to place the Bezier curve
     */
    double z_axis_;

};

