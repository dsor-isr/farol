/** 
 *  @file   DockingRFilter.h 
 *  @brief  Horizontal Filter DSORLab 
 *  @author Ravi Regalo
 *  @date   
 ***********************************************/
#ifndef CATKIN_WS_DOCKINGRFILTER_H
#define CATKIN_WS_DOCKINGRFILTER_H


// @.@ ROS Libraries
#include <ros/ros.h>
// @.@ TF's
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// @.@ ROS Messages
#include <auv_msgs/NavigationStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>

// @.@ Third Party Libraries
#include <Eigen/Eigen>
#include <GeographicLib/GeoCoords.hpp>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <cmath> 
#include <deque>
#include "docking_utils.h"


/* -------------------------------------------------------------------------*/
/**
 * @brief Relative Filter for docking 
 * @note cenas
 */
/* -------------------------------------------------------------------------*/
class DockingRFilter{
    public:
        // @.@ Methods
        /* -------------------------------------------------------------------------*/
        /**
         * @brief  Contructor Horizontal Filter
         */
        /* -------------------------------------------------------------------------*/
        DockingRFilter();


        /* -------------------------------------------------------------------------*/
        /**
         * @brief  Desctructor Horizontal Filter
         */
        /* -------------------------------------------------------------------------*/
        virtual ~DockingRFilter() = default;
        
        
        void reset();
        void initialize(double initial_observation);
        void tune(double Dt, double process_noise, double measurement_noise);

        bool predict(); // It uses the measurement buffer so no arguments needed
        bool update(double measurement);

        // @.@ Kalman Filter Variables        
        double state_vec_;
        double state_cov_;
        double initial_state_vec_;
        double initial_state_cov_;
        double process_noise_;
        double measurement_noise_;
        double innovation_vector_;
        double innovation_matrix_;
        double mahalanobis_distance_;
        double outlier_threshold_;
        double K_;

        // @.@ General purpose variables
        ros::Time last_predict_time_;
        bool initialized_{false};  

        // for inputs
        std::deque<double> input_buffer_;  // Buffer to store incoming messages
        std::deque<double> input_time_buffer_;  // Buffer to store timestamps
        bool usbl_outlier_rejection_{false};
        int outlier_rejected_{0};


    private:
};
#endif //CATKIN_WS_DOCKINGFILTER_H

