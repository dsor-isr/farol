#pragma once
#include "PathFollowing.h"

/**
 * @brief Path following using Lapierre's algorithm for path following
 * Method1: based on the work of Samson (1993)
 * 
 * This algorithm support:
 *    Controls:
 *      - yaw-rate
 *      - surge
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
class Samson : public PathFollowing {

  public:
    /**
     * @brief Constructor method for the Path Following class
     * 
     * @param k1 The controller gain
     * @param k2 The controller gain
     * @param k3 The controller gain
     * @param theta The controller gain
     * @param k_delta The controller gain
     * @param surge_pub The ROS surge publisher
     * @param yaw_rate_pub The ROS yaw rate publisher
     * @param mode_client The ROS service client to change the mode of operation of the path
     * to be the closest point
     */
    Samson(double k1, double k2, double k3, double theta, double k_delta, ros::Publisher surge_pub, ros::Publisher yaw_rate_pub, ros::ServiceClient mode_client);
    
    /**
     * @brief  Method that given a vector of doubles, updates the gains of the controller
     * 
     * @param gains A vector of gains
     *
     * NOTE: The default order of the gains is k1, k2, k3, theta, k_delta
     * 
     * @return a boolean which represents the success of the operation
     */
    bool setPFGains(std::vector<double> gains) override;

    /**
     * @brief  Method that implements the path following control law 
     */
    void callPFController(double dt) override;

    /** 
     * @brief  Method to publish the data from the path following 
     */
    void publish_private() override;

    /**
     * @brief  Method to run in the first iteration of the path following algorithm
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

  private:

    /**
     * @brief Controller parameters for the Samson Path Following Algorithm 
     */
    const int num_gains_ = 5; 
    double k1_{0.0};
    double k2_{0.0};
    double k3_{0.0};
    double theta_{0.0};
    double k_delta_{0.0};
    
    /**
     * @brief Auxiliar variables used to have continuous angles, since
     * this algorithm requires that the angles are differentiable
     */
    double yaw_out_{0.0};
    double yaw_out_old_{0.0};
    double yaw_old_{0.0};
    
    double psi_out_{0.0};
    double psi_out_old_{0.0};
    double psi_old_{0.0};
    
    void smoothVehicleYaw();
    void smoothPathYaw();

    /**
     * @brief The desired references to publish 
     */
    double desired_surge_;
    double desired_yaw_rate_;
 
    /**
     * @brief ROS publishers to publish the data
     */
    ros::Publisher surge_pub_;
    ros::Publisher yaw_rate_pub_;
    
    /**
     * @brief ROS service to use the closest point to the path 
     */
    ros::ServiceClient mode_client_;
};
