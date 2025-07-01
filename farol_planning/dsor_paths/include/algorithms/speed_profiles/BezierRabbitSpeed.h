#pragma once

#include "Speed.h"
#include "Path.h"
#include <ros/ros.h>


/** 
 *  @brief     Class that implements a Bezier parametrized varying speed value requirement. 
 *             This class will receive in its constructor the desired velocity 
 *             for the speed in the path frame
 *  @details   This class is used as a part of the speeds  library
 *  @author    Gorka Monteiro
 *  @version   1.0a
 *  @date      2025
 *  @copyright MIT
 */
class BezierRabbitSpeed : public Speed {
  
  public:
    
    
    /**
     * @brief  Constructor for the ConstVehicleSpeed class. Receives as a parameter
     * a double which represents the desired speed of the vehicle in the inertial frame
     *
     * @param px, py x and y values of curve control points
     * @param Tf Final time of arrival
     */
    BezierRabbitSpeed(const Eigen::VectorXd &px, const  Eigen::VectorXd &py, const double Tf);

   /**
     * @brief  Method to get the desired velocity for the virtual target on the path
     * given the path parameter (given by the value of gamma)
     *
     * @param gamma  The path parameter
     * @param tangent_norm  The norm of the tangent to the path
     *
     * @return  A double with the desired speed
     */
    double getVd(double gamma, double tangent_norm) override;

    /**
     * @brief  Method to get the desired acceleration for the virtual target on the path
     * given the path parameter (given by the value of gamma)
     *
     * @param gamma  The value of the path parameter
     * @param tangent_norm  The norm of the tangent to the path
     *
     * @return  A double with the desired acceleration
     */
    double get_d_Vd(double gamma, double tangent_norm) override;
    
    /**
     * @brief  Method to get the default desired velocity for safety
     * when we are doing path following and want to have a backup value
     *
     * @param gamma  The value of the path parameter
     * @param tangent_norm  The norm of the tangent to the path in gamma
     *
     * @return  A double with the default desired speed
     */
    double getDefaultVd(double gamma, double tangent_norm) override;

    /**
     * @brief  Method to compute the derivative of a bezier curve
     *
     * @param P original Control Points
     *
     * @return  New control Points
     */
    Eigen::VectorXd bezier_derivative(const Eigen::VectorXd &P);

    /**
     * @brief  Method to compute the de Casteljau's algorithm
     *
     * @param u Gamma value
     * @param pts Curve's Control points
     * @param i, j Auxiliary values for recursion
     *
     * @return  Point on the curve when gamma = u
     */
    double deCasteljau(double u, const Eigen::VectorXd &pts, int i, int j);

  private:
    
    /**
     * @brief  Attribute used to store the desired rabbit speed
     */
    double rabbit_speed_{0.0};

    /**
     * @brief  The default speed to be used if we have less speed sections then
     * path sections in the path. This is the value that will be used for the other 
     * path sections (the paths that do not have a corresponding speed sections)
     * and if this is the last speed section
     */
    double default_speed_{0.0};

    /**
     * @brief  Attributes used to store the squared vx and vy velocity control Points
     */
    Eigen::VectorXd d_x_;
    Eigen::VectorXd d_y_;
    Eigen::VectorXd dd_x_;
    Eigen::VectorXd dd_y_;

};
