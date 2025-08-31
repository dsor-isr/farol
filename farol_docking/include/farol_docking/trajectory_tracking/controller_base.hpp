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
#include <sophus/se3.hpp>

inline Eigen::Matrix3d hat(const Eigen::Vector3d& w){
  Eigen::Matrix3d K; 
  K <<      0, -w.z(),  w.y(),
         w.z(),     0, -w.x(),
        -w.y(),  w.x(),     0;
  return K;
}
inline Eigen::Vector3d vee(const Eigen::Matrix3d& K){
  return {K(2,1), K(0,2), K(1,0)};
}


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
    virtual void reset_xyz() = 0;
    virtual void reset_rpy() = 0;
    
    /**
    * @brief  Timer iteration callback
    *
    */
    virtual void compute_wrench(double dt) = 0;

    // >>> ADD: desired trajectory (world frame unless stated) <<<
    Eigen::Vector3d p_d_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d pd_d_{Eigen::Vector3d::Zero()};   // \dot p_d
    Eigen::Vector3d pdd_d_{Eigen::Vector3d::Zero()};  // \ddot p_d

    // Attitude reference: either yaw-only or full R_d & rates
    Eigen::Matrix3d R_d_{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d w_d_{Eigen::Vector3d::Zero()};    // \omega_d in R_d frame mapped to world via S(\omega_d)=R_d^T \dot R_d
    Eigen::Vector3d wdd_d_{Eigen::Vector3d::Zero()};  // \dot\omega_d (body of R_d)

    // Current (measured/estimated) state (be consistent in radians!)
    Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d R_{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d v_{Eigen::Vector3d::Zero()};      // body linear vel
    Eigen::Vector3d w_{Eigen::Vector3d::Zero()};      // body angular vel

    // Outputs
    Eigen::Vector3d force_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d torque_{Eigen::Vector3d::Zero()};


    ros::NodeHandle nh_, nh_private_;
};
