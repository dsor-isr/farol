/** 
 *  @file   DockingHFilter.h 
 *  @brief  Horizontal Filter DSORLab 
 *  @author Ravi Regalo
 *  @date   
 ***********************************************/
#ifndef CATKIN_WS_DOCKINGHFILTER_H
#define CATKIN_WS_DOCKINGHFILTER_H


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
class DockingHFilter{
    public:
        // @.@ Methods
        /* -------------------------------------------------------------------------*/
        /**
         * @brief  Contructor Horizontal Filter
         */
        /* -------------------------------------------------------------------------*/
        DockingHFilter();


        /* -------------------------------------------------------------------------*/
        /**
         * @brief  Desctructor Horizontal Filter
         */
        /* -------------------------------------------------------------------------*/
        virtual ~DockingHFilter() = default;

        void reset();
        void initialize(Eigen::Vector2d initial_observation);
        void tune(double Dt, Eigen::Matrix4d process_noise, Eigen::Matrix2d measurement_noise);
        
        
        bool predict(double yaw);
        bool update(Eigen::Vector2d measurement);
  


        // @.@ Public methods
        // bool computePredict(auv_msgs::NavigationStatus &state, const ros::Time &t_request);
        // void configure(DockingHFilter::config &configurations);
        // void newMeasurement(FilterGimmicks::measurement &m);
        // void deleteMeasurementsInBuffer();
        // void resetFilter();

        // @.@ Variables
        
        Eigen::Vector4d state_vec_;
        Eigen::Matrix4d state_cov_;
        Eigen::Vector4d initial_state_vec_;
        Eigen::Matrix4d initial_state_cov_;

        Eigen::Matrix4d process_noise_;
        Eigen::Matrix2d measurement_noise_;
        
        // Kalman filter variables
        Eigen::Matrix4d A_;
        Eigen::MatrixXd B_;
        Eigen::MatrixXd K_;
        Eigen::MatrixXd C_;
        Eigen::Vector2d innovation_vector_;
        Eigen::Matrix2d innovation_matrix_;
        double mahalanobis_distance_;
        double outlier_threshold_;


        bool initialized_;
        ros::Time last_predict_time_;
        double last_ahrs_;  

        ros::Time last_predict_;                ///< Time of last predict
        ros::Time last_update_;                 ///< Time of last update
        
        std::deque<Eigen::Vector2d> input_buffer_;  // Buffer to store incoming messages
        std::deque<double> input_time_buffer_;  // Buffer to store timestamps
        bool usbl_outlier_rejection_{false};
        int outlier_rejected_{0};


    private:
};
#endif //CATKIN_WS_DOCKINGRFILTER_H

