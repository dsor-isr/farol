#pragma once
#include "PathFollowing.h"

/**
 * @brief Path following using Pramod's algorithm for path following*
 *
 * This algorithm support:
 *    Controls:
 *      - yaw
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
class Pramod : public PathFollowing {

  public:

    /**
     * @brief Constructor method for the Path Following class
     *
     * @param gains A vector with the gains of the controller (2 gains only - Kp and Kd)
     * @param surge_pub The ROS surge publisher
     * @param yaw_pub The ROS yaw publisher
     * @param mode_client The ROS service client to change the mode of operation of the path (to the closest point)
     */
    Pramod(std::vector<double> gains, ros::Publisher surge_pub, ros::Publisher yaw_pub, ros::Publisher gamma_pub, ros::ServiceClient mode_client);
    
    /**
     * @brief  Method that given an array of doubles, updates the gains of the controller
     *
     * @param gains A vector of gains of the controller (only 2 gains Kp and Kd)
     *
     * @return a boolean which represents the success of the operation
     */
    bool setPFGains(std::vector<double> gains) override;

    /** 
     * @brief  Method that implements the path following control law 
     *
     * @param dt The time diference between the previous and current call (in seconds)
     */
    void callPFController(double dt) override;

    /**
     * @brief  Method to publish the data from the path following 
     */
    void publish_private() override;

    /**
     * @brief  Method used to start the algorithm in the first run
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
     * @brief Controller paramter gains 
     */
    std::vector<double> gains_;

    /**
     * @brief Variables to store the desired references
     */
    double desired_surge_{0.0};
    double desired_yaw_{0.0};

    /**
     * @brief ROS publishers
     */
    ros::Publisher surge_pub_;
    ros::Publisher yaw_pub_;
    ros::Publisher gamma_pub_;
    
    /**
     * @brief ROS service to use the closest point to the path 
     */
    ros::ServiceClient mode_client_;

};
