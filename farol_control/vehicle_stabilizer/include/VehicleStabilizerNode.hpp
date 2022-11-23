#pragma once

#include <ros/ros.h>

#include <map>

/* Auxiliary function from farol stack */
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <std_msgs/Float64.h>

class VehicleStabilizerNode {

public:

    /**
     * @param nh The public ROS nodehandle
     * @param nh_p The private ROS nodehandle
     */
    VehicleStabilizerNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p);

    /**
     * Destructor of the VehicleStabilizerNode class
     */
    ~VehicleStabilizerNode();

private:

    /**
     * @brief Nodle Handler attributes
     */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_;

    /**
     * @brief ROS Timer attributes 
     */
    ros::Timer timer_;

    /**
     * @brief References to publish to the inner-loops
     */
    std::map<std::string, double> references_;
    std::map<std::string, ros::Publisher> publishers_;

    /**
     * @brief ROS publishers to send references 
     */
    
    // Linear speed references
    ros::Publisher surge_ref_pub_;
    ros::Publisher sway_ref_pub_;
    ros::Publisher heave_ref_pub_;
    
    // Orientation references
    ros::Publisher roll_ref_pub_;
    ros::Publisher pitch_ref_pub_;
    ros::Publisher yaw_ref_pub_;

    // Angular velocity references
    ros::Publisher roll_rate_ref_pub_;
    ros::Publisher pitch_rate_ref_pub_;
    ros::Publisher yaw_rate_ref_pub_;

    // Depth and altitude references
    ros::Publisher altitude_ref_pub_;
    ros::Publisher depth_ref_pub_;

    // Default null value
    XmlRpc::XmlRpcValue default_null_ = 999999.9;

    /**
     * @brief Methods to initialize ROS
     */
    void initializeROSPublishers();
    void initializeParameters();
    void initializeTimer();
    double nodeFrequency();

    /**
     * @brief Method where the logic is located in order to update the control law
     *
     * @param event  A TimerEvent from ros
     */
    void timerIterCallback(const ros::TimerEvent &event);
};
