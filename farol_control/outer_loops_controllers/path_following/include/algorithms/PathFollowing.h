#pragma once

#include <Eigen/Core>
#include <vector>

/* Auxiliary definitions for the path state and vehicle state */
#include "States.h"

/* ROS includes for publishing the values and setting the mode of operation */
#include "ros/ros.h"
#include "dsor_paths/SetMode.h"
#include <std_msgs/Float64.h>
#include <farol_msgs/mPFollowingDebug.h>

/* Contains auxiliary functions for angle wrap-up and others */
#include <farol_gimmicks_library/FarolGimmicks.h>

/**
 * @brief  A Base class to update the path following law 
 *
 * @author    Marcelo Jacinto
 * @author    Joao Quintas
 * @author    Joao Cruz
 * @author    Hung Tuan
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
class PathFollowing {
  
  public:

   
    /**
     * @brief Virtual destructor for the abstract pathfollowing class
     */
    virtual ~PathFollowing();

    /**
     * @brief  Method to update the path following control law
     *
     * @param dt The time diference between the current and previous call (in seconds)
     */
    virtual void callPFController(double dt) = 0;
    
    /**
     * @brief  Method to publish the data given by the algorithm
     */
    void publish();
    
    virtual void publish_private() = 0;

    /**
     * @brief  Method used to setup the algorithm in the first iteration
     */
    virtual void start() = 0;

    /**
     * @brief  Method used to check whether we have reached the end of the path following
     * algorithm or not. This method will be called in every iteration of the algorithm,
     * and when it return true, the algorithm will stop
     *
     * @return  A boolean that represents whether we have reached the end
     */
    virtual bool stop() = 0;

    /**
     * @brief  Method used to reset the algorithm control parameters 
     * when running the algorithm more than once
     *
     * @return  Whether the reset was made successfully or not
     */
    virtual bool reset() = 0;

    /**
     * @brief Method to reset the virtual target of the vehicle (gamma)
     * to a pre-specified value. Not all controllers need this (example: Samson, Fossen which use the closest point)
     * 
     * @return Whether the reset was made successfully or not
     */
    virtual bool resetVirtualTarget(float value);

    /**
     * @brief Method to reset the virtual target of the vehicle (gamma)
     * to zero. Not all controllers need this (example: Samson, Fossen which use the closest point)
     * 
     * This method calls the resetVirtualTarget(float value) method which can be overriden by each pf controller
     * @return Whether the reset was made successfully or not
     */
    bool resetVirtualTarget();

    /** 
     * @brief  Receives a vector of gains that should be mapped to the specific controller
     * gains.
     *
     * This method must be implemented by each Path Following class
     *
     * @param gains A vector of gains for the controller
     */
    virtual bool setPFGains(std::vector<double> gains) = 0;

    /** 
     * @brief  Method to update the vehicle state used by the controller 
     *
     * @param vehicle_state A structure with the current state of the vehicle
     */
    void UpdateVehicleState(const VehicleState &vehicle_state);

    /**
     * @brief  Method to update the path state used by the controller 
     *
     * @param path_state A structure with the current state of the path
     */
    void UpdatePathState(const PathState &path_state);

    /**
     * @brief Method to set common publishers
     *
     *
    */
    void setPFollowingDebugPublisher(const ros::Publisher &pfollowing_debug_pub) {pfollowing_debug_pub_ = pfollowing_debug_pub;};
    
  protected:
   
    /**
     * @brief Variable to store the state of the vehicle 
     */
    VehicleState vehicle_state_;
    
    /** 
     * @brief Variable to store the state of the path 
     */
    PathState path_state_;  
    
    /** 
     * @brief Variable to store the state of the path 
     */
    PFollowingDebug pfollowing_debug_;  
    
    ros::Publisher pfollowing_debug_pub_; 
    
    /**
     * @brief Auxiliar method to smooth out the angle to be used by path 
     * following algorithms 
     */
    double algConvert(double alg_new, double alg_old, double alg_out_old);
};


