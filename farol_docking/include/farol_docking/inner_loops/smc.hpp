#pragma once

#include <vector>
#include <bitset>
#include <algorithm>
#include <Eigen/Core>
#include <sophus/se3.hpp>

// ros libraries
#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>  
#include <geometry_msgs/Point.h>  
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <auv_msgs/NavigationStatus.h>
#include <auv_msgs/BodyForceRequest.h>
#include <dsor_msgs/Measurement.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/mUSBLFix.h>

// farol libraries
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <farol_docking/utils/docking_utils.hpp>  
#include <farol_docking/inner_loops/controller_base.hpp>  


class SMC : public ControllerBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  SMC(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);
  

  ~SMC() = default;

  void set_gains_callback();
    
  /**
   * @brief  Every Controller should have some type of configuration function
   *
   */
  void configure();
  
  
  /**
  * @brief  Controller for position in R3
  *
  */
  bool compute_force(double Dt);

  /**
  * @brief  Controller for attitude in SO3
  *
  */
  bool compute_torque(double Dt);

  double gains_;
  ros::Subscriber sub_gains_;
  ros::Publisher debug_pub;
};

