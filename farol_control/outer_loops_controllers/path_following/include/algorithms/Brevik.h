#pragma once

#include "PathFollowing.h"
/**
 * @brief Path following using Brevik's algorithm for path following
 * Method4: based on the work of Brevik and Fossen (2005) 
 * This algorithm support:
 *    Controls:
 *      - yaw
 *      - surge
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
class Brevik : public PathFollowing {

  public:

    /**
     * @brief  Constructor method for the Path Following class
     * 
     * @param surge_pub The ROS surge publisher
     * @param yaw_pub The ROS yaw publisher
     * @param rabbit_pub The ROS rabbit publisher
     */
    Brevik(ros::Publisher surge_pub, ros::Publisher yaw_pub, ros::Publisher rabbit_pub);

    /**
     * @brief  Method to update the gains, given a vector of doubles 
     *
     * @param gains The gains for the controller
     *
     * NOTE: In this class the setPFGains method does nothing to the controller as all the gains
     * are fixed
     *
     * @return By default just returns false for this algorithm
     */
    bool setPFGains(std::vector<double> gains) override;
   
    /**
     * @brief  Method that implements the path following control law
     *
     * @param dt The time difference between last call and current call (in seconds)
     */
    void callPFController(double dt) override;

    /**
     * @brief  Method to publish the data from the path following 
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
     * @brief The desired references to publish
     */
    double desired_surge_{0.0};
    double desired_yaw_{0.0};
    
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
     * @brief The values for the dynamics of the gamma (virtual rabbit) 
     */
    double gamma_dot_{0.0};
    double gamma_{0.0};

    /**
     * @brief ROS publishers to publish the data
     */
    ros::Publisher surge_pub_;
    ros::Publisher yaw_pub_;
    ros::Publisher rabbit_pub_;
};

