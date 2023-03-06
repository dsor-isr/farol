#pragma once

#include <math.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <vector>

/* Include the messages used by publishers */
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <path_following/RefBypass.h> 

/* Messages used to receive data from the vehicle and from the path */
#include <auv_msgs/NavigationStatus.h>
#include <dsor_paths/PathData.h>
#include <farol_msgs/mPFollowingDebug.h>

/* Include the Control libraries */
#include "RelativeHeading.h"
#include "Marcelo.h"
#include "Aguiar.h"
#include "Brevik.h"
#include "Fossen.h"
#include "Romulo.h"
#include "Lapierre.h"
#include "Pramod.h"
#include "Samson.h"
#include "PathFollowing.h"
#include "States.h"

/* Include the generated services for the path following */
#include "path_following/ResetVT.h"
#include "path_following/SetPF.h"
#include "path_following/StartPF.h"
#include "path_following/StopPF.h"
#include "path_following/UpdateGainsPF.h"

/* Include the waypoint service to use when a mission finishes */
#include "waypoint/sendWpType1.h"
#include <std_srvs/Trigger.h>

/** 
 * Include the service to set the path mode to closest point (needed for
 * some algorithms)
 */
#include "dsor_paths/ResetPath.h"
#include "dsor_paths/SetMode.h"

/* Define Constants used when starting and stoping the path following */
#define WP_FINISH -3
#define FLAG_IDLE 0
#define FLAG_PF 6

/**
 * @brief     Path Following Node, where the magic happens 
 * @author    Marcelo Jacinto
 * @author    Joao Quintas
 * @author    Joao Cruz
 * @author    Hung Tuan
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
class PathFollowingNode {

  public:

    /**
     * @brief  The constructor of the path following node
     *
     * @param nh  The public ROS node handle
     * @param nh_p The private ROS node handle
     */
    PathFollowingNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p);

    /**
     * @brief The destructor of the path following node (where the subscribers, publishers
     * and services are terminated)
     */
    ~PathFollowingNode();

  private:

    /**
     * @brief Pointer to the Path Following algorithm to use 
     */
    PathFollowing *pf_algorithm_{nullptr};

    /**
     * @brief Auxiliar variables to store the current vehicle state and path values 
     */
    VehicleState vehicle_state_;
    PathState path_state_;
    bool flag_mode_ = false;

    /**
     * @brief Auxiliary variables to check whether we have received the first information
     * for intial path position and vehicle state
     */
    bool has_received_vehicle_state{false};
    bool has_received_path_state{false};

    /**
     * @brief Use to store the time-elapsed to be used by the controller
     */
    ros::Time prev_time_;

    /**
     * @brief Nodle Handler attributes
     */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_;

    /**
     * @brief ROS Services for path following 
     */
    ros::ServiceServer pf_start_srv_;
    ros::ServiceServer pf_stop_srv_;
    ros::ServiceServer pf_update_gains_srv_;

    ros::ServiceServer pf_aguiar_srv_;
    ros::ServiceServer pf_brevik_srv_;
    ros::ServiceServer pf_fossen_srv_;
    ros::ServiceServer pf_romulo_srv_;
    ros::ServiceServer pf_lapierre_srv_;
    ros::ServiceServer pf_pramod_srv_;
    ros::ServiceServer pf_samson_srv_;
    ros::ServiceServer pf_marcelo_srv_;
    ros::ServiceServer pf_relative_heading_srv_;

    /* Service to reset the virtual target position */
    ros::ServiceServer pf_reset_vt_srv_;

    /**
     * @brief ROS Services auxiliary to the path following 
     */
    ros::ServiceClient set_path_mode_client_;
    ros::ServiceClient wp_standard_client_;
    ros::ServiceClient reset_path_client_;
    ros::ServiceClient dr_reset_client_;         

    /**
     * @brief ROS publishers attributes
     */
    std::vector<ros::Publisher> publishers_;
    ros::Publisher flag_pub_;

    /**
     * @brief ROS subscribers attributes
     */
    std::vector<ros::Subscriber> subscribers_;
    ros::Subscriber state_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber vc_sub_;
    ros::Subscriber flag_sub_;
    ros::Subscriber flag_mode_sub_;

    /**
     * @brief ROS Timer attributes 
     */
    ros::Timer timer_;

    /**
     * @brief Method to allocate memory for a default PF controller class 
     */
    PathFollowing *getDefaultControllerLapierre();
    PathFollowing *getDefaultControllerBrevik();
    PathFollowing *getDefaultControllerAguiar();

    /**
     * @brief Method to delete the current controller being used
     */
    void deleteCurrentController();

    /**
     * @brief Method to stop the current path_following 
     */
    void stopAlgorithm();

    /**
     * @brief Method to initialize the ROS part
     */
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void initializeTimer();
    double nodeFrequency();

    /**
     * @brief Method where the logic is located in order to update the control law
     *
     * @param event  A TimerEvent from ros
     */
    void timerIterCallback(const ros::TimerEvent &event);

    /**
     * @brief Callbacks
     */
    void vcCallback(const std_msgs::Float64 &msg);
    void pathStateCallback(const dsor_paths::PathData &msg);
    void vehicleStateCallback(const auv_msgs::NavigationStatus &msg);
    void flagCallback(const std_msgs::Int8 &msg);
    void modeCallback(const std_msgs::Bool &msg);

    /**
     * @brief Services callbacks 
     */
    bool StartPFService(path_following::StartPF::Request &req,
        path_following::StartPF::Response &res);
    bool StopPFService(path_following::StopPF::Request &req,
        path_following::StopPF::Response &res);
    bool UpdateGainsPFService(path_following::UpdateGainsPF::Request &req,
        path_following::UpdateGainsPF::Response &res);

    bool SetRelativeHeadingService(path_following::SetPF::Request &req, 
        path_following::SetPF::Response &res);
    bool SetMarceloService(path_following::SetPF::Request &req, 
        path_following::SetPF::Response &res);
    bool SetAguiarService(path_following::SetPF::Request &req,
        path_following::SetPF::Response &res);
    bool SetBrevikService(path_following::SetPF::Request &req,
        path_following::SetPF::Response &res);
    bool SetFossenService(path_following::SetPF::Request &req, 
        path_following::SetPF::Response &res);
    bool SetRomuloService(path_following::SetPF::Request &req,
        path_following::SetPF::Response &res);
    bool SetLapierreService(path_following::SetPF::Request &req,
        path_following::SetPF::Response &res);
    bool SetPramodService(path_following::SetPF::Request &req,
        path_following::SetPF::Response &res);
    bool SetSamsonService(path_following::SetPF::Request &req,
        path_following::SetPF::Response &res);

    bool ResetVirtualTargetService(path_following::ResetVT::Request &req,
        path_following::ResetVT::Response &res);

    /**
     * @brief Service to send a waypoint when the path following stops
     */
    void sendWaypoint(double value);
    
    /**
     * @brief Service to reset the DeadReckoning position when the path following stops
     */
    void sendResetDeadReckoning();
};
