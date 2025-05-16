/** 
 *  @file   controller_base.hpp
 *  @brief  Docking Controller Algorithm header file
 *  @author Ravi Regalo ravi.regalo@tecnico.ulisboa.pt Instituto Superior Tecnico
 *  @date   
*/
#pragma once
#include <Eigen/Dense>
#include <ros/ros.h>
#include <auv_msgs/NavigationStatus.h>

class ControllerBase {
public:
    ControllerBase(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private) {}

    virtual ~ControllerBase() = default;
    
    /**
     * @brief  Every Controller should have some type of configuration function
     *
     */
    virtual void configure() = 0;
    
    /**
    * @brief  Timer iteration callback
    *
    */
    virtual bool compute_force(double Dt) = 0;
    virtual bool compute_torque(double Dt) = 0;

    /**
     * @brief  set the reference
     * @param[in] ref value of the reference
     * @param[in] type could be position or attitude
     *
     */
    virtual void setReference(const Eigen::Vector3d& ref, std::string type) {
        if(type == "position")
            position_ref_ =ref;
        if(type == "attitude")
            attitude_ref_ =ref;
    }

    /**
    * @brief  Timer iteration callback
    *
    */
    virtual void setState(const auv_msgs::NavigationStatus &msg) {
        // message if from docking filter
        if(msg.header.frame_id.find("dock") != std::string::npos){
          position_ << msg.local_position.x,msg.local_position.y,msg.local_position.z;  
          attitude_ << msg.local_attitude.roll,msg.local_attitude.pitch, msg.local_attitude.yaw;
        }else{
          position_ << msg.position.north,msg.position.east,msg.position.depth;  
          attitude_ << msg.orientation.x,msg.orientation.y, msg.orientation.z;
        }
        
        linear_velocity_ << msg.seafloor_velocity.x, msg.seafloor_velocity.y, msg.seafloor_velocity.z; 
        angular_velocity_ <<  msg.orientation_rate.x,msg.orientation_rate.y,msg.orientation_rate.z; 
      }
    
    
    ros::NodeHandle nh_, nh_private_;

    Eigen::Vector3d force_, torque_;
    Eigen::Vector3d position_ref_, attitude_ref_;
    Eigen::Vector3d position_, attitude_, linear_velocity_, angular_velocity_;

};
