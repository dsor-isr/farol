#pragma once

#include "PathFollowing.h"

/**
 * @brief Path following using Aguiar's algorithm for path following
 * Method6: based on the work of Aguiar and Hespanha (2007)  
 * This algorithm support:
 *    Controls:
 *      - yaw-rate
 *      - surge
 *      - virtual-target (gamma)
 *    Supports Cooperative Path Following - True
 *    Contains Currents Observers - True
 * 
 * @author    Marcelo Jacinto
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
class Marcelo : public PathFollowing {

  public:

    /**
     * @brief  Constructor method for the Path Following class
     *
     * @param delta Control gain
     * @param kk    Control gains
     * @param kz    Control gain
     * @param k_pos Observer gain
     * @param k_currents Observer gain
     * @param surge_pub The ROS surge publisher
     * @param yaw_rate_pub The ROS yaw rate publisher
     * @param rabbit_pub The ROS rabbit publisher
     */
    Marcelo(double delta, double kk[2], double kz, double k_pos, double k_currents, double rd[3], double d[3], ros::Publisher surge_pub, ros::Publisher yaw_rate_pub, ros::Publisher rabbit_pub, ros::Publisher currents_estimation_x_pub, ros::Publisher currents_estimation_y_pub);

    /**
     * @brief  Method to update the gains, given a vector of doubles 
     * 
     * @param gains A vector of gains to update in the controller
     * The default order is: delta, kk[0], kk[1], kz, k_pos, k_currents
     *
     * @return a boolean which represents the success of the operation
     */
    bool setPFGains(std::vector<double> gains) override;
   
    /**
     * @brief  Method that implements the path following control law
     *
     * @param dt The time step between last call and current call (in seconds)
     */
    void callPFController(double dt) override;

    /**
     * @brief  Method to publish_private the data from the path following 
     */
    void publish_private() override;

    /**
     * @brief  Method used in the first run to do initial setup
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
     * @brief Controller parameters for the Aguiar Path Following Algorithm
     */
    Eigen::Vector2d epsilon_;
    Eigen::Matrix2d delta_;
    Eigen::Matrix2d delta_inv_;
    Eigen::Matrix2d kk_;
    double kz_;

    /**
     * @brief Observer gains for the currents 
     */
    double k_currents_;
    double k_pos_;
    bool first_iteration_{true};

    /** 
     * @brief The desired references to publish
     */
    double desired_surge_{0.0};
    double desired_yaw_rate_{0.0};

    /**
     * @brief Vector to old the estimator for the ocean currents 
     */
    Eigen::Vector2d pos_hat_{0.0, 0.0};
    Eigen::Vector2d currents_hat_{0.0, 0.0};
    
    /** 
     * @brief The values for the dynamics of the gamma (virtual rabbit) 
     */
    double gamma_ddot_{0.0};
    double gamma_dot_{0.0};
    double gamma_{0.0};

    /**
     * @brief The values for the offset on the path following
     */
    Eigen::Vector3d rd_{0.0, 0.0, 1.0};
    Eigen::Vector3d d_{0.0, 0.0, 0.0};

    /**
     * @brief ROS publishers to publish the data 
     */
    ros::Publisher surge_pub_;
    ros::Publisher yaw_rate_pub_;
    ros::Publisher rabbit_pub_;
    ros::Publisher currents_estimation_x_pub_;
    ros::Publisher currents_estimation_y_pub_;
};

