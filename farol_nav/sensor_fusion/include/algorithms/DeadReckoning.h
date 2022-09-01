/** 
 *  @file   DeadReckoning.h 
 *  @brief  DeadReckoning DSORLab 
 *  @author DSOR ISR/IST
 *  @date   2021-09-09 
 *  @note   don't you miss the danger
 ***********************************************/
#ifndef CATKIN_WS_DEADRECKONING_H
#define CATKIN_WS_DEADRECKONING_H

// @.@ ROS Libraries
#include <ros/ros.h>

// @.@ ROS Messages
#include <auv_msgs/NavigationStatus.h>
#include <farol_msgs/mState.h>
#include <dsor_msgs/Measurement.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>
#include <math.h>

// @.@ Third Party Libraries
#include <farol_gimmicks_library/FarolGimmicks.h>

#define RAD2DEG(x) x*180.0/FarolGimmicks::PI
#define DEG2RAD(x) x*FarolGimmicks::PI/180.0

/* -------------------------------------------------------------------------*/
/**
 * @brief  DeadReckoning class
 *
 * @note This node only outputs useful information when dvl is present
 * @note Every time a new mission is deployed (FLAG=6), the DR is reseted to the 
 * current navigation state(x,y)
 * */
/* -------------------------------------------------------------------------*/
class DeadReckoning{
  public:

    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Contructor DeadReckoning
     */
    /* -------------------------------------------------------------------------*/
    DeadReckoning(ros::NodeHandle *nh, ros::NodeHandle *nh_private);


    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Desctructor DeadReckoning
     */
    /* -------------------------------------------------------------------------*/
    ~DeadReckoning();

    // @.@ Public methods

    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Propagate the state to the current time
     *
     * @returns Success or Failure 
     */
    /* -------------------------------------------------------------------------*/
    void computePredict();

  private:
    ros::NodeHandle nh_, nh_private_;

    // @.@ Handy Variables
    bool initialized_{false};                     ///< Initialized DR estimate
    bool p_dvl_body_frame_{true};                 ///< Identify dvl frame
    double dvl_vx_{0.0};                          ///< Inertial velocity vx from dvl
    double dvl_vy_{0.0};                          ///< Inertial velocity vy from dvl
    double roll_{0.0};                            ///< roll measurement from ahrs
    double pitch_{0.0};                           ///< pitch measurement from ahrs
    double yaw_{0.0};                             ///< yaw measurement from ahrs

    // @.@ Subscribers
    ros::Subscriber sub_velocity_;                ///< Subscriber for DVL
    ros::Subscriber sub_orientation_;             ///< Subscriber for AHRS
    ros::Subscriber sub_true_state_;              ///< Subscriber for State
    ros::Subscriber flag_sub_;                    ///< Subscriber for FLag, indicates when mission starts

    ros::Publisher state_dr_pub_;                 ///< Publishers for DR estimate
    ros::Publisher state_dr_console_pub_;         ///< Publishers for DR estimate to console

    ros::ServiceServer reset_filter_dr_srv_;      ///< Service for reset DR position

    auv_msgs::NavigationStatus state_dr_msg_;     ///< message to be published by state_dr_pub_
    farol_msgs::mState state_dr_console_;         ///< message to be published by state_dr_console_pub_

    // @.@ States Vectores
    std::vector<double> state_dr_{0.0 ,0.0};      ///< 2D state_dr(x,y)
    std::vector<double> true_state_{0.0 ,0.0};    ///< 2D state to store navigation filter state(x,y)

    // @.@ Timers
    ros::Time last_predict_;                     ///< Time of last dr calculation

    // @.@ Encapsulation the gory details of initializing subscribers, publishers and services
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief Initialize Subscribers 
     */
    /* -------------------------------------------------------------------------*/
    void initializeSubscribers();
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief Iniatialize Publishers 
     */
    /* -------------------------------------------------------------------------*/
    void initializePublishers();
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief Initialize Services
     */
    /* -------------------------------------------------------------------------*/
    void initializeServices();

    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Intialize the filter 
     *
     * @param m Measurement from a sensor or Initial state defined in config file
     *
     * @returns   
     */
    /* -------------------------------------------------------------------------*/
    bool initialize();

    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Predict for DR
     *
     * @param state_vec for dr
     * @param dt period
     */
    /* -------------------------------------------------------------------------*/
    void predict(std::vector<double> &state_vec, double dt);

    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief Receive velocity measurements from DVL
     *
     * @param msg dvl 
     */
    /* -------------------------------------------------------------------------*/
    void velocityCallback(const dsor_msgs::Measurement &msg);
    

    /* -------------------------------------------------------------------------*/
    /**
     * @brief Recevice orientation measurements from AHRS/IMU
     *
     * @param msg ahrs
     */
    /* -------------------------------------------------------------------------*/
    void orientationCallback(const dsor_msgs::Measurement &msg);
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief Receive state from the navigation filter
     *
     * @param msg navigation state
     */
    /* -------------------------------------------------------------------------*/
    void stateCallback(const auv_msgs::NavigationStatus &msg);
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief Flag that indicates when a mission starts. Reset DR to current 
     * navigation filter state(x,y)
     *
     * @param msg wait for a 6
     */
    /* -------------------------------------------------------------------------*/
    void flagCallback(const std_msgs::Int8 &msg);
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Service to reset the DR estimate to the current navigation filter state
     *
     * @param req 
     * @param res
     *
     * @returns  bool 
     */
    /* -------------------------------------------------------------------------*/
    bool resetDRService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

};
#endif //CATKIN_WS_DEADRECKONING_H