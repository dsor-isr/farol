# pragma once

#include <stdlib.h>
#include <vector>
#include <optional>

/* ROS specific includes */
#include <ros/ros.h>
#include <std_msgs/Float64.h>

/* Include the message for publishing the path information */
#include <dsor_paths/PathData.h>
#include <auv_msgs/NavigationStatus.h>
#include <farol_msgs/mState.h>

/* Farol gimicks library for reading configuration paramters */
#include <farol_gimmicks_library/FarolGimmicks.h>

/* Include the algorithms library for the Paths */
#include "Path.h"
#include "PathSection.h"
#include "Arc2D.h"
#include "Bernoulli.h"
#include "Circle2D.h"
#include "Line.h"
#include "ConstRabbitSpeed.h"
#include "ConstVehicleSpeed.h"

/* Include the generated services for the paths */
#include "dsor_paths/ResetPath.h"
#include "dsor_paths/SetMode.h"
#include "dsor_paths/SpawnArc2D.h"
#include "dsor_paths/SpawnBernoulli.h"
#include "dsor_paths/SpawnCircle2D.h"
#include "dsor_paths/SpawnLine.h"

/* Services for setting the speed profile of the vehicle*/
#include "dsor_paths/SetConstSpeed.h"


/** 
 *  @brief     Implementation of the PathNode. Creates a Path, adds elements 
 *             to the path and publishes the path data when listening to the 
 *             path parameter gamma
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class PathNode {
  public:
    
    /**
     * @brief  Constructor of the PathNode class
     *
     * @param nh  Pointer to a public ROS NodeHandle
     * @param nh_p  Pointer to a private ROS NodeHandle
     */
    PathNode(ros::NodeHandle* nh, ros::NodeHandle *nh_p);

    /**
     * @brief  Destructor of the PathNode class
     */
    ~PathNode();

  private:

    /** 
     * @brief Path structure that will have all the logic 
     */
    std::optional<double> gamma_; // The current gamma being published
    Path * path_{NULL}; // A pointer to the path object

    /**
     * @brief Frame_id and seq for messages 
     */
    uint32_t seq_{0};
    std::string frame_id_;


    /**
     * @brief Auxiliar variables to store the current vehicle position - usefull
     * if we want the closest point to the path 
     */
    Eigen::Vector3d vehicle_pos_;
    bool closer_point_mode_{false};

    /** 
     * @brief ROS NodeHandlers attributes
     */
    ros::NodeHandle nh_; 
    ros::NodeHandle nh_p_;

    /**
     * @brief ROS Subscribers attributes 
     */ 
    ros::Subscriber gamma_sub_;
    ros::Subscriber vehicle_sub_;

    /** 
     * @brief ROS Publishers attributes 
     */
    ros::Publisher path_pub_;
    ros::Publisher virtual_target_pub_;

    /** 
     * @brief ROS Services 
     */
    ros::ServiceServer reset_path_srv_;
    ros::ServiceServer set_mode_srv_;
    ros::ServiceServer arc2d_srv_;
    ros::ServiceServer bernoulli_srv_;
    ros::ServiceServer circle2D_srv_;
    ros::ServiceServer line_srv_;
    ros::ServiceServer rabbit_const_speed_srv_;
    ros::ServiceServer vehicle_const_speed_srv_;

    /**
     * @brief ROS Timers attributes 
     */
    ros::Timer timer_; 

    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void initializeTimer();
    void loadParams();
    void shutdownServices();

    double nodeFrequency();

    /**
     * @brief Method where is the logic for publishing the path information 
     */
    void timerIterCallback(const ros::TimerEvent &event); 

    /** 
     * @brief Callbacks 
     */
    void gammaCallback(const std_msgs::Float64 &msg);
    void vehicleStateCallback(const auv_msgs::NavigationStatus &msg);

    /** 
     * @brief Services Callbacks 
     */
    bool ResetPathService(dsor_paths::ResetPath::Request &req, dsor_paths::ResetPath::Response &res);
    bool SetModeService(dsor_paths::SetMode::Request &req, dsor_paths::SetMode::Response &res);
    bool Arc2DService(dsor_paths::SpawnArc2D::Request &req, dsor_paths::SpawnArc2D::Response &res); 
    bool BernoulliService(dsor_paths::SpawnBernoulli::Request &req, dsor_paths::SpawnBernoulli::Response &res);
    bool Circle2DService(dsor_paths::SpawnCircle2D::Request &req, dsor_paths::SpawnCircle2D::Response &res);
    bool LineService(dsor_paths::SpawnLine::Request &req, dsor_paths::SpawnLine::Response &res);
    bool RabbitConstSpeedService(dsor_paths::SetConstSpeed::Request &req, dsor_paths::SetConstSpeed::Response &res);
    bool VehicleConstSpeedService(dsor_paths::SetConstSpeed::Request &req, dsor_paths::SetConstSpeed::Response &res);

    /** 
     * @brief Auxiliar method to be called inside the callbacks
     */
    bool loadSectionIntoPath(PathSection * section);
    bool loadSpeedIntoPath(Speed * speed); 
};
