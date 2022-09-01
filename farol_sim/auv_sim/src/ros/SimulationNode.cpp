#include "SimulationNode.h"
#include <nav_msgs/Odometry.h>

SimulationNode::SimulationNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
        : nh_(*nh), nh_p_(*nh_p) {

    /* Initialize all the ROS subscribers, publisher, services and update timer */
    ROS_INFO("In class constructor of SimulationNode");

    /* Initialize subscribers, publishers and services */
    this->initializeSubscribers();
    this->initializePublishers();
    this->initializeServices();

    /* Create the AUV simulation class */
    this->auv_ = this->instantiateAUV();

    /* Start the simulation */
    this->initializeTimer();
}

SimulationNode::~SimulationNode() {

    /* Shutdown all the subscribers */
    this->shutdownSubscribers();

    /* Shutdown all the publishers */
    this->shutdownPublishers();

    /* Shutdown all the services */
    this->shutdownServices();

    /* Stop the timer */
    this->timer_.stop();

    /* Shutdown the ros node */
    this->nh_.shutdown();
}

std::unique_ptr<AUV> SimulationNode::instantiateAUV() {

    /* Read the parameters from the parameter server */
    unsigned int num_thrusters;
    double mass, fluid_density, zg, vehicle_density, thrusters_max_input, thrusters_min_input, thrusters_gain, thrusters_pole, thrusters_delay, sampling_frequency;
    std::vector<double> inertia_tensor, linear_damping_tensor, quadratic_damping_tensor, added_mass_tensor, allocation_matrix, lump_param_positive, lump_param_negative, disturbance_mean, disturbance_sigma, disturbance_minimum, disturbance_maximum;

    /* Load the parameters for the dynamics of the AUV */
    this->nh_p_.param<std::string>("frame_id", this->frame_id_, "simulated_AUV");
    this->nh_p_.getParam("fluid_density", fluid_density);
    this->nh_p_.getParam("vehicle/mass", mass);
    this->nh_p_.getParam("vehicle/zg", zg);
    this->nh_p_.getParam("vehicle/vehicle_density", vehicle_density);
    this->nh_p_.getParam("vehicle/inertia_tensor", inertia_tensor);
    this->nh_p_.getParam("vehicle/linear_damping_tensor", linear_damping_tensor);
    this->nh_p_.getParam("vehicle/quadratic_damping_tensor", quadratic_damping_tensor);
    this->nh_p_.getParam("vehicle/added_mass_tensor", added_mass_tensor);

    /* Load the parameters for the actuators of the AUV */
    this->nh_p_.getParam("vehicle/actuators/allocation_matrix", allocation_matrix);
    this->nh_p_.getParam("vehicle/actuators/lump_param_positive", lump_param_positive);
    this->nh_p_.getParam("vehicle/actuators/lump_param_negative", lump_param_negative);
    this->nh_p_.getParam("vehicle/actuators/max_input", thrusters_max_input);
    this->nh_p_.getParam("vehicle/actuators/min_input", thrusters_min_input);
    this->nh_p_.getParam("vehicle/actuators/gain", thrusters_gain);
    this->nh_p_.getParam("vehicle/actuators/pole", thrusters_pole);
    this->nh_p_.getParam("vehicle/actuators/delay", thrusters_delay);
    this->nh_p_.getParam("node_frequency", sampling_frequency);

    /* Compute the number of thrusters of the allocation matrix */
    num_thrusters = allocation_matrix.size() / 6;

    /* Check if the number of thrusters is correct, or if there were rounding errors */
    if (num_thrusters * 6 != allocation_matrix.size())
        throw "Allocation matrix for thrusters not well defined. Simulation will not continue!";
    ROS_INFO_STREAM("Simulated AUV will have: " << num_thrusters << " thrusters.");

    /* Update the vector that will store the published desired thrust to apply to the vehicle accordingly */
    this->desired_thrust_.resize(num_thrusters);

    /* Load the parameters for the simulated ocean disturbances */
    this->nh_p_.getParam("current/mean", disturbance_mean);
    this->nh_p_.getParam("current/sigma", disturbance_sigma);
    this->nh_p_.getParam("current/minimum", disturbance_minimum);
    this->nh_p_.getParam("current/maximum", disturbance_maximum);

    /* Return a new AUV object */
    return std::make_unique<AUV>(mass, fluid_density, zg, vehicle_density,
                     Eigen::Vector3d(inertia_tensor.data()),
                     Eigen::Matrix<double, 6, 1>(linear_damping_tensor.data()),
                     Eigen::Matrix<double, 6, 1>(quadratic_damping_tensor.data()),
                     Eigen::Matrix<double, 6, 1>(added_mass_tensor.data()),
                     Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>>(allocation_matrix.data(), num_thrusters, 6),
                     Eigen::Vector3d(lump_param_positive.data()),
                     Eigen::Vector3d(lump_param_negative.data()),
                     Eigen::Vector2d(thrusters_min_input, thrusters_max_input),
                     thrusters_gain,
                     thrusters_pole,
                     thrusters_delay,
                     1.0 / sampling_frequency,
                     Eigen::Vector3d(disturbance_mean.data()),
                     Eigen::Vector3d(disturbance_sigma.data()),
                     Eigen::Vector3d(disturbance_minimum.data()),
                     Eigen::Vector3d(disturbance_maximum.data()));

    /* NOTE: Check if these values can actually be copied by reference to the eigen structures (specially the allocation matrix). If some bug occurs later down the road, this is the first place we should check */
}

void SimulationNode::initializeTimer() {
   this->timer_ = this->nh_.createTimer(ros::Duration(1.0 / SimulationNode::nodeFrequency()), &SimulationNode::timerUpdateCallback, this);
}

double SimulationNode::nodeFrequency() {

   double node_frequency;

   /* Read from parameter server the desired node frequency (if not found, default to 50 Hz) */
    this->nh_p_.param<double>("node_frequency", node_frequency, 50);

    ROS_INFO("SimulationNode will run at : %lf [hz]", node_frequency);
    return node_frequency;
}

void SimulationNode::timerUpdateCallback(const ros::TimerEvent &event) {

    /* Get the difference between previous update time and disturbance update time */
    static ros::Time prev_time = ros::Time::now();
    ros::Time curr_time = ros::Time::now();
    ros::Duration dt = curr_time - prev_time;
    prev_time = curr_time;

    /* Update the AUV state (if the simulation is not paused) */
    if(!this->paused_) this->auv_->update(dt.toSec(), this->desired_thrust_);

    /* Get the disturbance state of the AUV */
    State curr_state = this->auv_->getState();

    /* Publish the disturbance state of the AUV */
    nav_msgs::Odometry odom_msg = nav_msgs::Odometry();
    odom_msg.header.stamp = curr_time;
    odom_msg.header.frame_id = this->frame_id_;

    odom_msg.pose.pose.position.x = curr_state.eta1(0);
    odom_msg.pose.pose.position.y = curr_state.eta1(1);
    odom_msg.pose.pose.position.z = curr_state.eta1(2);

    // Create a quaternion according to ZYX order of rotation
    Eigen::Quaternion<double> orientation;
    orientation = Eigen::AngleAxisd(curr_state.eta2(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(curr_state.eta2(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(curr_state.eta2(0), Eigen::Vector3d::UnitX());

    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();

    odom_msg.twist.twist.linear.x = curr_state.v1(0);
    odom_msg.twist.twist.linear.y = curr_state.v1(1);
    odom_msg.twist.twist.linear.z = curr_state.v1(2);

    odom_msg.twist.twist.angular.x = curr_state.v2(0);
    odom_msg.twist.twist.angular.y = curr_state.v2(1);
    odom_msg.twist.twist.angular.z = curr_state.v2(2);

    this->odometry_pub_.publish(odom_msg);
}

int main(int argc, char ** argv) {

    /* Initiate ros and get the public and private node-handles */
    ros::init(argc, argv, "SimulationNode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("main: instantiating an object of type SimulationNode");

    /* Instantiate the Simulation Node*/
    SimulationNode simulation(&nh, &nh_p);

    /* Going into spin and let the timer callback do all the work */
    ros::spin();

    return 0;
}