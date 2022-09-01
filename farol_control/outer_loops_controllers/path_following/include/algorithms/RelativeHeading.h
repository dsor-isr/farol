#pragma once
#include "PathFollowing.h"

/**
 * @brief Path following using RelativeHeading's algorithm for path following*
 *
 * This algorithm support:
 *    Controls:
 *      - yaw
 *      - surge
 *      - sway
 *    Supports Cooperative Path Following - True
 *    Contains Currents Observers - False
 *
 * @author    Francisco Rego
 * @author    Andre Potes
 * @author    Marcelo Jacinto
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
class RelativeHeading : public PathFollowing {

public:

    /**
     * @brief  Constructor method for the Path Following class
     *
     * @param surge_pub The ROS surge publisher
     * @param sway_pub The ROS sway publisher
     * @param yaw_pub The ROS yaw publisher
     * @param rabbit_pub The ROS rabbit publisher
     */
    RelativeHeading(double kx, double ky, double kz, const Eigen::Vector2d &p_sat, double yaw_offset, ros::Publisher surge_pub, ros::Publisher sway_pub, ros::Publisher yaw_pub, ros::Publisher rabbit_pub);

    /**
     * @brief  Method to update the gains, given a vector of doubles
     *
     * @param gains A vector of gains to update in the controller
     * The default order is: TODO
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
     * @brief Controller parameters for the RelativeHeading Path Following Algorithm
     */
    double kx_;
    double ky_;
    double kz_;
    Eigen::Vector2d p_sat_;
    double yaw_offset_;

    /**
     * @brief The desired references to publish
     */
    double desired_surge_{0.0};
    double desired_sway_{0.0};
    double desired_yaw_{0.0};

    /**
     * @brief The values for the dynamics of the gamma (virtual rabbit)
     */
    double gamma_ddot_{0.0};
    double gamma_dot_{0.0};
    double gamma_{0.0};

    /**
     * @brief ROS publishers to publish the data
     */
    ros::Publisher surge_pub_;
    ros::Publisher sway_pub_;
    ros::Publisher yaw_pub_;
    ros::Publisher rabbit_pub_;
};

