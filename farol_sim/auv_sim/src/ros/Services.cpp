#include <SimulationNode.h>

void SimulationNode::initializeServices() {

    /* Get the services' topic names */
    std::string start_pause_service_name;
    this->nh_p_.getParam("topics/services/start_pause", start_pause_service_name);

    /* Initiate the service servers */
    this->start_pause_srv_ = this->nh_.advertiseService(start_pause_service_name, &SimulationNode::startPauseServiceCallback, this);
}

void SimulationNode::shutdownServices() {

    /* Shutdown start-pause service */
    this->start_pause_srv_.shutdown();
}


/* Start and Pause simulation - service callback*/
bool SimulationNode::startPauseServiceCallback(auv_sim::StartPause::Request &req, auv_sim::StartPause::Response &res) {

    /* Switch the simulation state from paused to running and vice-versa */
    this->paused_ = (this->paused_ == true) ? false : true;

    /* Return success to the user */
    std::string running_status_string = (this->paused_ == true) ? "PAUSED" : "RUNNING";
    ROS_INFO_STREAM("Simulation is: " << running_status_string);
    res.success = true;
    return true;
}