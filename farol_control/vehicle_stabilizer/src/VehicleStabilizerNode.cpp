#include "VehicleStabilizerNode.hpp"

/* Class Constructor for the MQTTBridgeNode */
VehicleStabilizerNode::VehicleStabilizerNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p) : nh_(*nh), nh_p_(*nh_p) {

    ROS_INFO("in class constructor of VehicleStabilizerNode");

    /* Initialize the ROS side */
    this->initializeParameters();
    this->initializeROSPublishers();
    this->initializeTimer();
}

/* Class Destructor */
VehicleStabilizerNode::~VehicleStabilizerNode() {

    /* Stop the timer callback */
    this->timer_.stop();

    /* Shutdown the node */
    this->nh_.shutdown();
}

/**
 * @brief Initialize all the parameters to define which references to publish
 */
void VehicleStabilizerNode::initializeParameters() {

    /* Read the references to apply from the parameter server */
    XmlRpc::XmlRpcValue references_dict = FarolGimmicks::getParameters<XmlRpc::XmlRpcValue>(this->nh_p_, "references");

    /* Make sure we have received a dictionary */
    if(references_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct) {

        /* Iterate over the dictionary */
        for(std::map<std::string, XmlRpc::XmlRpcValue>::iterator p = references_dict.begin(); p!=references_dict.end(); ++p) {
            
            /* Get the key of the dictionary */
            std::string key = p->first;

            /* Get the value of the dictionary */
            double value = p->second;

            /* Create the dictionary of references to apply */
            this->references_[key] = value;
        }
    }
}

/**
 * @brief Initialize all publishers
 */
void VehicleStabilizerNode::initializeROSPublishers() {

    ROS_INFO("Initializing Publishers for VehicleStabilizerNode");

    /* Iterate over all the references that we want to publish */
    for(std::map<std::string, double>::iterator it=this->references_.begin(); it!=this->references_.end(); ++it) {
        
        try {
            /* Attempt at getting the topic name for the publishers for each of the pre-defined reference */
            this->publishers_[it->first] = this->nh_.advertise<std_msgs::Float64>(
                FarolGimmicks::getParameters<std::string>(
                    this->nh_p_, std::string("topics/publishers/") + it->first), 1);
        } catch(...) {
            ROS_WARN_STREAM("Could not get topics/publishers/" << it->first << "which we will publish Float64 to...");
            ROS_WARN_STREAM("Node is shutting down - fix the configurations yamls");
            ros::shutdown();
        }
        
    }
}

/**
 * @brief  Initialize the timer callback
 */
void VehicleStabilizerNode::initializeTimer() {
  this->timer_ = this->nh_.createTimer(ros::Duration(1.0 / VehicleStabilizerNode::nodeFrequency()), &VehicleStabilizerNode::timerIterCallback, this);
  this->timer_.start();
}

/**
 * @brief  Setup the Node working frequency
 */
double VehicleStabilizerNode::nodeFrequency() {

  double node_frequency = FarolGimmicks::getParameters<double>(this->nh_p_, "node_frequency", 10);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

/**
 * @brief Method that will publish periodically the references that we wish to call
 *
 * @param event  A TimerEvent from ros
 */
void VehicleStabilizerNode::timerIterCallback(const ros::TimerEvent &event) {

    std_msgs::Float64 msg;

    // Iterate over the map of publishers
    for(std::map<std::string, ros::Publisher>::iterator it=this->publishers_.begin(); it!=this->publishers_.end(); ++it) {

        msg.data = this->references_[it->first];

        // Publish the desired reference
        this->publishers_[it->first].publish(msg);
    }
}

////////////////////////////////////////////////////////////////////
//////////////////////// MAIN //////////////////////////////////////
////////////////////////////////////////////////////////////////////
/**
 * @brief  The main function - the entry point of this node
 *
 * @param argc  The argument count
 * @param argv  The argument array
 */
int main(int argc, char **argv) {

    ros::init(argc, argv, "vehicle_stabilizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("main: instantiating an object of type VehicleStabilizerNode");

    /* Instantiate the PathFollowing Node*/
    VehicleStabilizerNode VehicleStabilizerNode(&nh, &nh_p);

    /* Going into spin and let the timer callback do all the work */
    ros::spin();

    return 0;
}