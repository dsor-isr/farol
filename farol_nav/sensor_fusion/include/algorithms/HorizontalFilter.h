/** 
 *  @file   HorizontalFitler.h 
 *  @brief  Horizontal Filter DSORLab 
 *  @author DSOR ISR/IST
 *  @date   2021-09-09 
 *  @note   don't you miss the danger
 ***********************************************/
#ifndef CATKIN_WS_HORIZONTALFILTER_H
#define CATKIN_WS_HORIZONTALFILTER_H



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
#include "FilterGimmicks.h"

typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;

/* -------------------------------------------------------------------------*/
/**
 * @brief  Horizontal Filter class
 * @note This Class estimates the state of the vehicle in the horizontal frame. 
 * The state includes position, velocity, acceleration and currents.
 * The latest state estimate of the filter is obtained using the 
 * exposed function getEstimate().
 * Measurement updates to the filter is done either using measCallback() 
 * and the latest state estimate is accessed using getEstimate().
 * Water currents are estimated but the output is not exposed to the user.
 */
/* -------------------------------------------------------------------------*/
class HorizontalFilter{
public:
const int static MEAS_LEN = 6;
const int static STATE_LEN = 8;

// ############################
// @.@ STRUCT to configure filter object
// ############################
struct config{
    bool initialized{false}, broadcast_tf;
    double reject_counter[4], init_cov[4], process_noise[4], kalman_config[3];
    std::string base_frame, odom_frame, map_frame, world_frame; 
    std::string name_vehicle_id;

    FilterGimmicks::measurement meas_init; // initializing frame and/or measurement
    std::vector<FilterGimmicks::measurement> sensors; // input sensor list
    tf2_ros::TransformBroadcaster *br_node;
    // Constructors
};

/* -------------------------------------------------------------------------*/
/**
 * @brief  Contructor Horizontal Filter
 */
/* -------------------------------------------------------------------------*/
HorizontalFilter();


/* -------------------------------------------------------------------------*/
/**
 * @brief  Desctructor Horizontal Filter
 */
/* -------------------------------------------------------------------------*/
virtual ~HorizontalFilter() = default;

// @.@ Public methods

/* -------------------------------------------------------------------------*/
/**
 * @brief  Propagate the state to the current time
 *
 * @param state State vector 
 * @param t_request current time
 *
 * @returns Success or Failure 
 */
/* -------------------------------------------------------------------------*/
bool computePredict(auv_msgs::NavigationStatus &state, const ros::Time &t_request);

/* -------------------------------------------------------------------------*/
/**
 * @brief  Configure filter variables and set initialization conditions
 *
 * @param configurations
 */
/* -------------------------------------------------------------------------*/
void configure(HorizontalFilter::config &configurations);

/* -------------------------------------------------------------------------*/
/**
 * @brief  Tranforms the measurement from the sensor frame to the filter world frame.
 * Calls addMeasurement and forwardPropagation methods
 *
 * @param msg New measurement
 */
/* -------------------------------------------------------------------------*/
void newMeasurement(FilterGimmicks::measurement &m);

/* -------------------------------------------------------------------------*/
/**
 * @brief  Clears all measurements older than a timer period defined in save_meas_interval
 */
/* -------------------------------------------------------------------------*/
void deleteMeasurementsInBuffer();

/* -------------------------------------------------------------------------*/
/**
 * @brief  Returns the currents
 *
 * @returns x_current, y_current
 */
/* -------------------------------------------------------------------------*/
std::vector<double> getExtimateCurrents();


/* -------------------------------------------------------------------------*/
/**
 * @brief  Reset horizontal filter
 */
/* -------------------------------------------------------------------------*/
void resetFilter();

private:

// @.@ Handy Variables
bool initialized_;                      ///< Initialized Filter 
std::list<FilterGimmicks::measurement> meas_list_;  ///< measurement list

// @.@ Kalman Variables
double t_save_measurement_;             ///< period to save measurements in the list
double t_reset_;                        ///< reset time for filter, if no measurement received
Vec state_vec_;                         ///< State vector
Mat state_cov_;                         ///< Covariance matrix
Mat process_cov_;                       ///< Process covariance
Vec state_reject_counter_;              ///< for outlier rejection
Vec reject_counter_;                    ///< keep track of outliers

// @.@ Transformation Variables
bool tf_broadcast_flag_;                ///< Flag to switch TF broadcast
std::string  world_frame_id_;           ///< world frame id
std::string init_frame_id_;             ///< init frame id
std::string name_vehicle_id_;
tf2_ros::TransformBroadcaster tf_broadcast_;  ///< transform broadcaster tf
    
// @.@ Time
ros::Time last_predict_;                ///< Time of last predict
ros::Time last_update_;                 ///< Time of last update


//  @.@ Kalman Functions

/* -------------------------------------------------------------------------*/
/**
 * @brief  Intialize the filter 
 *
 * @param m Measurement from a sensor or Initial state defined in config file
 *
 * @returns   
 */
/* -------------------------------------------------------------------------*/
bool initialize(FilterGimmicks::measurement& m);

/* -------------------------------------------------------------------------*/
/**
 * @brief  Predict step Kalman filter
 *
 * @param state_vec State vector
 * @param state_cov Covariance matrix
 * @param dt period
 */
/* -------------------------------------------------------------------------*/
void predict(Vec& state_vec, Mat& state_cov, double dt);


/* -------------------------------------------------------------------------*/
/**
 * @brief  Update step Kalman filter
 *
 * @param state_vec State vector
 * @param state_cov Matrix covariance
 * @param m Measurement
 */
/* -------------------------------------------------------------------------*/
bool update(Vec& state_vec, Mat& state_cov, std::list<FilterGimmicks::measurement>::iterator &it_measurement);

// @.@ Auxillary Functions

/* -------------------------------------------------------------------------*/
/**
 * @brief  Add a new measurement to a ordered buffer according to the time stamp
 *
 * @param m Measurement
 *
 * @returns Iterator that points to the message before the new one. 
 *
 */
/* -------------------------------------------------------------------------*/
std::list<FilterGimmicks::measurement>::iterator addMeasurementToBuffer(FilterGimmicks::measurement& m) ;

/* -------------------------------------------------------------------------*/
/**
 * @brief  Progate the state and the covariance until measurement time and then
 * update
 *
 * @param it_measurement Measurment from a buffer  
 */
/* -------------------------------------------------------------------------*/
void forwardPropagation(std::list<FilterGimmicks::measurement>::iterator it_measurement);

};
#endif //CATKIN_WS_HORIZONTALFILTER_H