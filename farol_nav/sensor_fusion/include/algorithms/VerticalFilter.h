/** 
 *  @file   VerticalFitler.h 
 *  @brief  Vertical Filter DSORLab 
 *  @author DSOR ISR/IST
 *  @date   2021-09-09 
 *  @note   don't you miss the danger
 ***********************************************/

#ifndef CATKIN_WS_VERTICALFILTER_H
#define CATKIN_WS_VERTICALFILTER_H

#include <algorithm>
#include <stdlib.h>
// ROS Libraries
#include <ros/ros.h>
// TF's
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// ROS Messages
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <auv_msgs/NavigationStatus.h>
#include "FilterGimmicks.h"

// Third Party Libraries
#include <Eigen/Eigen>
typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;



/* -------------------------------------------------------------------------*/
/**
 * @brief  This Class estimates the state of the vehicle in the horizontal 
 * frame. The state includes depth, velocity, altitude, and bouyancy.
 * The latest state estimate of the filter is obtained using the exposed 
 * function getEstimate().
 * Measurement updates to the filter is done either using measCallback().
 * 
 * @Note Bouyancy is estimated but the output is not exposed to the user.
 * 
 */
/* -------------------------------------------------------------------------*/
class VerticalFilter
{
public:
    static const int MEAS_LEN = 3;    ///< Measurement length 
    //static const int STATE_LEN = 4;   ///< State Length

    // @.@ STRUCT to configure filter object
    struct config
    {
        bool initialized{false}, broadcast_tf;
        double vertical_drag[3];        ///< alpha, beta, bouyancy
        double reject_counter[MEAS_LEN], init_cov[MEAS_LEN], process_noise[MEAS_LEN], kalman_config[3];
        std::string base_frame, odom_frame, map_frame, world_frame; 
        std::string name_vehicle_id;

        FilterGimmicks::measurement meas_init;
        std::vector<FilterGimmicks::measurement> sensors;
        tf2_ros::TransformBroadcaster *br_node;
        // Constructors
    };
    
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Constructor
     */
    /* -------------------------------------------------------------------------*/
    VerticalFilter();

    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Destructor
     */
    /* -------------------------------------------------------------------------*/
    virtual ~VerticalFilter() = default;

    // @.@ Public methods

    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Configure filter program variables and set initialization conditions 
     *
     * @param config_ structure with configurations
     *
     */
    /* -------------------------------------------------------------------------*/
    void configure(const VerticalFilter::config configurations);
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Propagate the state to the current time
     *
     * @param state State vector
     * @param t_request current time
     *
     */
    /* -------------------------------------------------------------------------*/
    void computePredict(auv_msgs::NavigationStatus &state, ros::Time t_request);

    /* -------------------------------------------------------------------------*/
    /**
     * @brief Checks and processes a new measurement 
     *
     * @param m Measurement
     *
     * @returns true if measurement is processed successfully 
     */
    /* -------------------------------------------------------------------------*/
    void newMeasurement(const FilterGimmicks::measurement &m);
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Resets the filter, will require re-initialization to start back update
     * again
     *
     */
    /* -------------------------------------------------------------------------*/
    void resetFilter();

private:
    // @.@ Program Variables

    //  +.+ Handy Variables
    bool initialized_{false};    /// < Initialized Filter
    
    //  +.+ Kalman Variables
    double alpha_, beta_, buoyancy_;	///< Parameters for vertical dynamic model
    double t_period_;									///< filter prediction perido
    double t_save_measurement_;				///< measurments older than this time are rejected. Note: not being used
    double t_reset;										///< Reset filter, if after t_reset no measurements
    double t_wait_altimeter_ = 2.0;   ///< During the initicalization wait 2sec for a altimeter measurement other wise will initiate with a depth measurement
    Vec state_vec_;										///< State vector
    Mat state_cov_;										///< Covariance matrix
    Mat process_cov_;									///< Process covariance
    Vec state_reject_counter_;				///< For outlier rejection
		Vec reject_counter_;							///< Keep track of outliers

    //  +.+ Transformation Variables
    bool broadcast_tf_;								///< Flag to switch TF broadcast
    std::string world_frame_id_;			///< world frame id
    std::string name_vehicle_id_;
    tf2_ros::TransformBroadcaster br_;///< transform brodcaster tf

    //  @.@ Timers
    ros::Time last_predict_;					///< Time of last predict
		ros::Time	last_update_;						///< Time of last update
    ros::Time init_with_depth_;       ///< Time until we wait for a altimeter measurement


    //  @.@ Kalman Functions

    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Initialize vertical filter
     *
     * @param m Measurement from a sensor or Initial state defined in config file
     *
     * @returns Success or Failure bool 
     */
    /* -------------------------------------------------------------------------*/
    bool initialize(const FilterGimmicks::measurement &m);

    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief Predict phase Kalman Filter
     *
     * @param state_vec_ State vector
     * @param state_cov_ State covariance
     * @param dt Time step
     *
     */
    /* -------------------------------------------------------------------------*/
    void predict(Vec &state_vec, Mat &state_cov, double dt);

    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Update phase Kalman Filter
     *
     * @param state_vec_ State Vector
     * @param state_cov_ State Covariande Vector
     * @param m Measurement
     *
     */
    /* -------------------------------------------------------------------------*/
    void update(Vec &state_vec, Mat &state_cov, const FilterGimmicks::measurement &m);

    //  @.@ Auxillary Functions
    
    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Propagate the state and covariance until measurement time and then
     * update the state with update() method
     *
     * @param m Measurement
     */
    /* -------------------------------------------------------------------------*/
    void forwardPropagation(const FilterGimmicks::measurement &m);

};
#endif //CATKIN_WS_VERTICALFILTER_H