#pragma once

#include "PathFollowing.h"

/**
 * @brief Path following using Aguiar's algorithm modified to also control sway
 * instead of yaw_rate (by Romulo)
 *
 * This algorithm support:
 *    Controls:
 *      - surge
 *      - sway
 *      - virtual-target (gamma)
 *    Supports Cooperative Path Following - True
 *    Contains Currents Observers - False
 *
 * @author    Marcelo Jacinto
 * @author    Joao Quintas
 * @author    Joao Cruz
 * @author    Hung Tuan
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
class Romulo : public PathFollowing {

  public:

    /**
     * @brief Constructor method for the Path Following class
     *
     * @param gains The gains of the controller
     * @param surge_pub The ROS publisher of surge
     * @param sway_pub The ROS publisher of sway
     * @param rabbit_pub The ROS publisher of the virtual rabbit
     */
    Romulo(std::vector<double> gains, ros::Publisher surge_pub, ros::Publisher sway_pub, ros::Publisher rabbit_pub);

    /**
     * @brief  Method that given an array of doubles, updates the gains of the controller
     *
     * @param gains The gains of the controller
     *
     * @return a boolean which represents the success of the operation
     */
    bool setPFGains(std::vector<double> gains) override;

    /**
     * @brief  Method that implements the path following control law 
     *
     * @param dt The time difference between the current and previous call (in seconds)
     */
    void callPFController(double dt) override;
    
    /** 
     * @brief  Method to publish the data from the path following 
     */
    void publish_private() override;

    /**
     * @brief  Method to do initial setup for the first run
     */
    void start() override;

    /**
     * @brief  Method used to check whether we reached the end of the algorithm or not
     *
     * @return The success of the operation
     */
    bool stop() override;

    /**
     * @brief  Method used to reset the algorithm control parameters 
     * when running the algorithm more than once
     *
     * @return  Whether the reset was made successfully or not
     */
    bool reset() override;

    /**
     * @brief Method to reset the virtual target of the vehicle (gamma)
     * to a pre-specified value. Not all controllers need this (example: Samson, Fossen which use the closest point)
     * 
     * @return Whether the reset was made successfully or not
     */
    bool resetVirtualTarget(float value) override;
  
  private:
    
    /**
     * @brief Controller parameters for the FreeHeading Path Following algorithm
     */
    std::vector<double> gains_;
    Eigen::Matrix2d ke;
    double kz;

    /**
     * @brief The control references going to be generated 
     */
    double desired_surge_;
    double desired_sway_;
   
    /**
     * @brief The values for the dynamics of the gamma (virtual target) 
     */
    double gamma_ddot_{0.0};
    double gamma_dot_{0.0};
    double gamma_{0.0};

    /**
     * @brief ROS publishers to publish the data 
     */
    ros::Publisher surge_pub_;
    ros::Publisher sway_pub_;
    ros::Publisher rabbit_pub_;
};
