/** 
 *  @file   VerticalFitler.h 
 *  @brief  Vertical Filter DSORLab 
 *  @author DSOR ISR/IST
 *  @date   2021-09-09 
 *  @note   don't you miss the danger
 ***********************************************/
#ifndef CATKIN_WS_ROTATIONALFILTER_H
#define CATKIN_WS_ROTATIONALFILTER_H
/* --------------------------------------------------------------------------------

-------------------------------------------------------------------------------- */

#include <algorithm>

// ROS Libraries
#include <ros/ros.h>
// TF's
// #include <tf2/eigen.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// ROS Messages

#include <auv_msgs/NavigationStatus.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
// Third Party Libraries
#include <Eigen/Eigen>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include "FilterGimmicks.h"



typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;

/* -------------------------------------------------------------------------*/
/**
 * @brief  This Class estimates the state of the vehicle in the rotational frame. The state includes orientation and angular velocity.
The latest state estimate of the filter is obtained using the function computePredict().
Measurement updates to the filter is done using newMeasurement().

 */
/* -------------------------------------------------------------------------*/
class RotationalFilter{
public:
// @.@ Define state length
static const int STATE_LEN = 6;
double PI = 3.1415926;

// @.@ STRUCT to configure filter object
struct config{
	  bool initialized{false};
    bool bypass_ahrs;
    double reject_counter[STATE_LEN/2];
    double init_cov[STATE_LEN], process_noise[STATE_LEN], kalman_config[3];
    std::string base_frame, odom_frame, map_frame, world_frame; 
    std::string name_vehicle_id;

    FilterGimmicks::measurement meas_init;
    std::vector<FilterGimmicks::measurement> sensors;
    tf2_ros::TransformBroadcaster *br_node;
};

/* -------------------------------------------------------------------------*/
/**
 * @brief  Rotational Filter Constructor
 */
/* -------------------------------------------------------------------------*/
RotationalFilter();


/* -------------------------------------------------------------------------*/
/**
 * @brief  Rotational Filter Destructor
 */
/* -------------------------------------------------------------------------*/
virtual ~RotationalFilter() = default;

// @.@ Public methods

/* -------------------------------------------------------------------------*/
/**
 * @brief  Progate the state to the time t_request
 *
 * @param state
 * @param t_request
 *
 * @note  Why have we used TF here?  We want: Rotation of base_link wrt world, i.e. header = world, child = base_link (TFwb). I have: 1. static_tf - Rotation of imu wrt base_link, i.e. header = base_link, child = imu (TFbi); 2. sensor output - Rotation of imu wrt world, i.e. header = world, child = imu (TFwi). Solution: TFwb = TFwi * (TFbi)^-1
 */
/* -------------------------------------------------------------------------*/
void computePredict(auv_msgs::NavigationStatus &state, const ros::Time &t_request);


/* -------------------------------------------------------------------------*/
/**
 * @brief configure filter program variables and may initializes the filter 
 *
 * @param configurations struct to store configurations from yaml file
 */
/* -------------------------------------------------------------------------*/
void configure(const config configurations);

/* -------------------------------------------------------------------------*/
/**
 * @brief  Checks and processes a new measurement
 *
 * @param m measurement
 */
/* -------------------------------------------------------------------------*/
void newMeasurement(const FilterGimmicks::measurement &m);

/* -------------------------------------------------------------------------*/
/**
 * @brief  Reset the horizontal filter
 */
/* -------------------------------------------------------------------------*/
void resetFilter();

private:

//  @.@ Handy Variables
bool initialized_{false};             ///< Initialized Filter 
bool ahrs_as_input_{false};           ///< Indicates if ahrs is input or measurement

//  @.@ Kalman Variables
double t_period_;                     ///< predict period
double t_save_measurement_;           ///< measurements older than this time are rejected. Note not being used 
double t_reset_;                      ///< Reset filter if after t_reset_ no measurements
Vec state_vec_;                       ///< State vector
Mat state_cov_;                       ///< Covariance Matrix 
Mat process_cov_;                     ///< Process Covariance
Vec state_reject_counter_;            ///< for outlier rejection
Vec reject_counter_;                  ///< Keep track of outliers

//  @.@ Transformation Variables
std::string base_frame_id_;           ///< base frame 
std::string odom_frame_id_;           ///< odom frame
std::string map_frame_id_;            ///< map frame
std::string world_frame_id_;          ///< world frame
std::string name_vehicle_id_;
tf2_ros::Buffer tf_buffer_;           ///< tf buffer
tf2_ros::TransformListener* tf_listener_; ///< tf listener
tf2_ros::TransformBroadcaster tf_broadcast_; ///< transform broadcast tf

//  @.@ Time
ros::Time last_predict_;              ///< time of last predict 
ros::Time last_update_;               ///< time of last update


//  @.@ Kalman Methods

/* -------------------------------------------------------------------------*/
/**
 * @brief  Initialize rotational filter
 *
 * @param m Measurement from a sensor or Initial value of state defined in config file
 *
 * @returns Success or failure   
 */
/* -------------------------------------------------------------------------*/
bool initialize(const FilterGimmicks::measurement& m);

/* -------------------------------------------------------------------------*/
/**
 * @brief  Predict step of Kalman filter. Predicts the state and the covariance
 *
 * @param state_vec State
 * @param state_cov Covariance matrix
 * @param dt Time step
 *
 */
/* -------------------------------------------------------------------------*/
void predict(Vec& state_vec, Mat& state_cov, double dt);

/* -------------------------------------------------------------------------*/
/**
 * @brief  Kalman update step. Updates the state and covariance with a measurement
 *
 * @param state_vec State vector
 * @param state_cov Covariance matrix
 * @param m measurement
 */
/* -------------------------------------------------------------------------*/
void update(Vec& state_vec, Mat& state_cov, const FilterGimmicks::measurement& m);

//  @.@ Auxillary Functions

/* -------------------------------------------------------------------------*/
/**
 * @brief  Propagate the state and the covariance until measurement time and 
 * then update the state
 *
 * @param m measurement
 */
/* -------------------------------------------------------------------------*/
void forwardPropagation(const FilterGimmicks::measurement& m);
};

#endif //CATKIN_WS_HORIZONTALFILTER_H