/**
 * @file   docking_filter_node.hpp
 * @brief  Docking Filter Node class
 * @author Ravi Regalo <ravi.regalo@tecnico.ulisboa.pt>
 * @date   2025-04-25
 * 
 * Description :)
 */
#pragma once

// usefull libraries
#include <vector>
#include <bitset>
#include <algorithm>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"

// ros libraries
#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>  
#include <geometry_msgs/Point.h>  
#include <geometry_msgs/Vector3.h>  
#include <dsor_msgs/Measurement.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/mUSBLFix.h>
#include <auv_msgs/NavigationStatus.h>
#include <sensor_msgs/Imu.h>
#include <topic_tools/shape_shifter.h>


// farol libraries
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <farol_docking/utils/docking_utils.hpp>  
#include <farol_docking/filter/docking_filter.hpp>  
#include <farol_docking/SetGain.h> 

struct UsblParams {
    // nominal noise (1σ)
    double sr = 0.000001;                  // m
    double sb = 0.000001 * M_PI/180.0;      // rad
    double se = 0.000001 * M_PI/180.0;      // rad
    // outlier noise (1σ)
    double SR = 0.5;                   // m
    double SB = 15.0 * M_PI/180.0;     // rad
    double SE = 15.0 * M_PI/180.0;     // rad
    // event probabilities (per sensor triplet)
    double p_outlier = 0.1;
    double p_dropout = 0.1;          
};

/**
 * @brief   Interface between ROS and docking filter algorithm
 */
class DockingFilterNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
  * @brief Constructor
  *
  * @param nodehandle
  * @param nodehandle_private
  */
 	DockingFilterNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /**
   * @brief  Destructor
   */
 	~DockingFilterNode();
  
 private:
  /**
   * @brief Load parameters from parameter server 
   */
  void loadParams();

  /**
   * @brief Initialize ROS node Subscribers
   */
  void initializeSubscribers();

  /**
   * @brief  Initialize ROS node Publishers
   */
 	void initializePublishers();
 
  /**
   * @brief  Initialize ROS node Services
   */
 	void initializeServices();

  /**
   * @brief Initialize ROS node Timer  
   */
 	void initializeTimer();

  /**
   * @brief Callback to handle measurement ROS messages. 
   *        They could be AHRS messages or DVL messages
   * @param msg ROS DSOR Measurement msg
   * 
   */
  void measurement_callback(const dsor_msgs::Measurement &msg);

  /**
   * @brief Callback to handle USBL measurement ROS messages. it waits for measurements 
   *        from the vehicle's own USBL and also the docking station USBL fix that comes 
   *        through the acoustims comms
   * @param msg ROS reset USBL fix message
   * 
   */
  void usbl_callback(const farol_msgs::mUSBLFix &msg);

  /**
   * @brief Callback that resets the filter to uninitialized state@
   * @param msg ROS reset msg
   */
  void reset_callback(const std_msgs::Empty &msg);

  void terrain_normal_callback(const geometry_msgs::Vector3 &msg);
  void imu_raw_callback(const sensor_msgs::Imu &msg);


  /**
   * @brief Function that runs at the node frequency, propagrating the state until 
   *        current time and sending the current state over to ROS
   *
   * @param event 
   */
  void timerIterCallback(const ros::TimerEvent& event);
  
  
  bool reconfigureNumericSrv(farol_docking::SetGain::Request& req, farol_docking::SetGain::Response& res);
  
  // ROS node handlers
 	ros::NodeHandle nh_;          
 	ros::NodeHandle nh_private_; 

 	// Subscribers
  ros::Subscriber sub_reset_;
  ros::Subscriber sub_velocity_;
  ros::Subscriber sub_orientation_;
  ros::Subscriber sub_position_;
  ros::Subscriber sub_usbl_fix_;
  ros::Subscriber sub_usbl_accoms_;
  ros::Subscriber sub_dock_inertial_pos_;
  ros::Subscriber sub_terrain_d_;
  ros::Subscriber sub_imu_raw_, sub_imu_raw_sim_;
  
  // Publishers
  ros::Publisher state_pub_, body_velocity_pub_;
  // ros::Publisher console_state_pub_;
  // ros::Publisher debug_pub_;

  // ROS Services
  ros::ServiceServer reconfig_numeric_srv_;
  
  // ROS messages
  auv_msgs::NavigationStatus state_msg_;
  
 	// ROS node iteration timer
 	ros::Timer timer_;    

  // ROS parameters
  double node_frequency_; 
  bool debug_, use_dvl_filt_in_controller_;
  bool use_terrain_{false};
  
  // Measurements stuff
  // USBL stuff
  std::bitset<4> usbl_state_;   // keeps track of all the messages that need to be received for an usbl set to be completed
  Sophus::Vector6d usbl_set_;   // holds the set of usbl measurement [auv(range, bearing, elevation), dock(range, bearing, elevation)]
  std::array<double, 4> usbl_times_; 
  ros::Time usbl_time_;   // timestamp from the last received usbl message, or a least the time of the latest measured thing
  Eigen::Vector3d dvl_velocity_, ahrs_velocity_;  // save dvl velocity


  // buffer to hold some usbl sets in the beginning in order to initalize 
  std::vector<Sophus::Vector6d> initializer_buffer_;
  std::string dock_frame_id_;
  
  // Filter Algorithm object
  std::unique_ptr<DockingFilter> docking_filter_;
  // to hold the state
  Sophus::SE3d state_;

  Eigen::Vector3d r_dvl_{0.45, 0.0, -0.2};

  bool realistify_;
  std::mt19937 rng_{std::random_device{}()};
  UsblParams P_; // tweak if you like

};

#include <array>
#include <random>
#include <cmath>

inline double wrapPi(double a) { return std::remainder(a, 2.0*M_PI); } // (-pi,pi]
inline double clampElev(double e){
    if(e >  M_PI/2) return  M_PI/2;
    if(e < -M_PI/2) return -M_PI/2;
    return e;
}



struct UsblFlags {
    bool reset  = false;  // any sensor asks for reset
    bool reset1 = false;  // sensor 1 asks for reset
    bool reset2 = false;  // sensor 2 asks for reset
};

// z = [r1,b1,e1,r2,b2,e2]
inline void realistify_usbl(std::array<double,6>& z,
                            std::mt19937& rng,
                            const UsblParams& P,
                            UsblFlags* flags = nullptr)
{
    auto bern = [&](double p){ return std::bernoulli_distribution(std::clamp(p,0.0,1.0))(rng); };
    auto nrm  = [&](double s){ return std::normal_distribution<double>(0.0, s)(rng); };

    for(int s=0; s<2; ++s){
        int i = 3*s; // r,b,e indices

        // emulate dropout without NaN: just request a reset
        if(bern(P.p_dropout)){
            if(flags){
                flags->reset = true;
                (s==0 ? flags->reset1 : flags->reset2) = true;
            }
            continue; // leave values unchanged; your code will reset/ignore
        }

        const bool out = bern(P.p_outlier);
        const double sr = out ? P.SR : P.sr;
        const double sb = out ? P.SB : P.sb;
        const double se = out ? P.SE : P.se;

        z[i+0] += nrm(sr);
        z[i+1]  = wrapPi(z[i+1] + nrm(sb));   // bearing
        z[i+2]  = clampElev(z[i+2] + nrm(se)); // elevation
    }
}
