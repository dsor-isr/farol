#include "SimulationNode.h"

void SimulationNode::initializeSubscribers() {

    /* Get the subscribers' topic names */
    std::string thrusters_topic;
    this->nh_p_.getParam("topics/subscribers/thrusters", thrusters_topic);

    /* Subscribe to the topics */
    this->thrusters_sub_ = nh_.subscribe(thrusters_topic, 1, &SimulationNode::updateDesiredThrustCallback, this);
}

void SimulationNode::shutdownSubscribers() {

    /* Shutdown thrusters subscriber */
    this->thrusters_sub_.shutdown();
}

void SimulationNode::updateDesiredThrustCallback(const dsor_msgs::Thruster &msg) {

    /* Check if the number of thrusters in the message coincides with the number of thrusters in the vehicle */
    if (this->auv_->getNumberThrusters() != msg.value.size()) {
        ROS_WARN_STREAM("Thruster message has " << msg.value.size() << " values, but simulated vehicle only has " << this->auv_->getNumberThrusters() << " thrusters.");
        return;
    }

    /* Save the desired thrust to apply to the vehicles */
    std::vector<double> data = msg.value;
    this->desired_thrust_ = Eigen::Map<Eigen::VectorXd>(data.data(), data.size());
}
