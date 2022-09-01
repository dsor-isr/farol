#include "SimulationNode.h"
#include <nav_msgs/Odometry.h>

void SimulationNode::initializePublishers() {

    /* Get the publishers' topic names */
    std::string odometry_topic;
    this->nh_p_.getParam("topics/publishers/odometry", odometry_topic);

    /* Subscribe to the topics */
    this->odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic, 1);
}

void SimulationNode::shutdownPublishers() {

    /* Shutdown Odometry publisher */
    this->odometry_pub_.shutdown();
}