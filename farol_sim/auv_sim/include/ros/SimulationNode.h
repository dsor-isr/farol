#pragma once

#include <memory>

#include <ros/ros.h>
#include <dsor_msgs/Thruster.h>
#include "auv_sim/StartPause.h"

#include <AUV.h>

/**
 * @brief SimulationNode class - implements the ROS interface to actually perform an AUV simulation
 * @author    Marcelo Jacinto
 * @version   1.0.0
 * @date      2021/11/12
 * @copyright MIT
 */
class SimulationNode {

public:
    SimulationNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p);
    ~SimulationNode();

private:

    /* SIMULATION LOGIC AND PARAMETERS */
    std::unique_ptr<AUV> auv_;
    std::unique_ptr<AUV> instantiateAUV();
    std::string frame_id_;

    /* Flag to start and pause the simulation */
    bool paused_{false};

    /* Desired thruster values to apply in the next iteration */
    Eigen::VectorXd desired_thrust_;

    /* ROS SPECIFIC INITIALIZATION ATTRIBUTES AND METHODS */
    ros::NodeHandle nh_, nh_p_;
    ros::Timer timer_;

    void initializeSubscribers();
    void shutdownSubscribers();

    void initializePublishers();
    void shutdownPublishers();

    void initializeServices();
    void shutdownServices();

    void initializeTimer();
    double nodeFrequency();

    /* Subscribers */
    ros::Subscriber thrusters_sub_;

    /* Publishers */
    ros::Publisher odometry_pub_;

    /* Services */
    ros::ServiceServer start_pause_srv_;

    /* Subscriber callbacks */
    void updateDesiredThrustCallback(const dsor_msgs::Thruster &msg);

    /* Service callbacks */
    bool startPauseServiceCallback(auv_sim::StartPause::Request &req, auv_sim::StartPause::Response &res);

    /* Timer callback - where the simulation code is called */
    void timerUpdateCallback(const ros::TimerEvent &event);

};
