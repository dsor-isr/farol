#pragma once
#include "PathFollowing.h"

/**
 * @brief Path following using Fossen's algorithm for path following
 * Method3: based on the work of Fossen(2015)
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
class Fossen : public PathFollowing {

  public:

    /**
     * @brief Constructor method for the Path Following class
     *
     * @param surge_pub The ROS surge publisher
     * @param yaw_pub   The ROS yaw publisher
     * @param mode_client The ROS client for the path (to change the mode of operation to closest point)
     */
    Fossen(ros::Publisher surge_pub, ros::Publisher yaw_pub, ros::ServiceClient mode_client);

    /**
     * @brief  Method that given an array of doubles, updates the gains of the controller
     *
     * @param gains The gains of the controller
     *
     * NOTE: In this controller this method does nothing, as there are no gains to tweak.
     * They are all fixed
     *
     * @return By default just returns false for this algorithm 
     */
    bool setPFGains(std::vector<double> gains) override;

    /** 
     * @brief  Method that implements the path following control law 
     *
     * @param dt The time diference between last call and current call (in seconds)
     */
    void callPFController(double dt) override;

    /**
     * @brief  Method to publish the data from the path following 
     */
    void publish_private() override;

    /**
     * @brief  Method used to start the algorithm in the first run
     *
     * @return the success of the operation
     */
    void start() override;

    /**
     * @brief  Method used to check whether we reached the end of the algorithm or not
     *
     * @return the success of the operation
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
     * @brief Variables to store the desired references 
     */
    double desired_surge_{0.0};
    double desired_yaw_{0.0};

    /**
     * @brief ROS publishers 
     */
    ros::Publisher surge_pub_;
    ros::Publisher yaw_pub_;

    /**
     * @brief ROS service to use the closest point to the path
     */
    ros::ServiceClient mode_client_;

    const int num_gains_ = 1;
    double Delta_h{2.0};
};
