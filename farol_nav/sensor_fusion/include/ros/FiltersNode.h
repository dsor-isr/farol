/** 
 *  @file   FiltersNode.h 
 *  @brief  Filter/sensor fusion DSORLab 
 *  @author DSOR ISR/IST
 *  @date   2021-09-09 
 *  @note   don't you miss the danger
 ***********************************************/

#ifndef CATKIN_WS_FILTERSNODE_H
#define CATKIN_WS_FILTERSNODE_H

#include "DeadReckoning.h"
#include "HorizontalFilter.h"
#include "VerticalFilter.h"
#include "RotationalFilter.h"
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// ROS Libraries
#include <ros/ros.h>
#include <fstream>

// ROS Messages and stuff
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <dsor_msgs/Measurement.h>
#include <auv_msgs/NavigationStatus.h>
#include <farol_msgs/mState.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include "sensor_fusion/SetVCurrentVelocity.h"

// @.@ Farol Messages
#include <farol_msgs/Currents.h>

// 3rd Parties
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <GeographicLib/UTMUPS.hpp>


#define RAD2DEG(x) x*180.0/FarolGimmicks::PI
#define DEG2RAD(x) x*FarolGimmicks::PI/180.0

/* -------------------------------------------------------------------------*/
/**
 * @brief  FiltersNode class
 *
 * @note Read all the parameters for each filter(Horicontal, Rotations, Vertical),
 * and configures them.
 * Takes care of the iteration using timer.
 * Also responsible for controlling the measurement buffer of the horizontal filter.
 */
/* -------------------------------------------------------------------------*/
class FiltersNode
{
public:
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Constructor
   *
   * @param nh node handle
   * @param nh_private node handle private
   */
  /* -------------------------------------------------------------------------*/
	FiltersNode(ros::NodeHandle *nh, ros::NodeHandle *nh_private);

	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Destructor 
   */
  /* -------------------------------------------------------------------------*/
  ~FiltersNode();

	// @.@ Public methods
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Defines the node frequency 
   *
   * @returns Node frequency  
   */
  /* -------------------------------------------------------------------------*/
  double nodeFrequency();

private:
	ros::NodeHandle nh_, nh_private_;           ///< Node handles
	
	// @.@ Subsctibers
	ros::Subscriber sub_position_;               ///< Measurement position subscriber
  ros::Subscriber sub_velocity_;               ///< Measurement velocity subscriber
  ros::Subscriber sub_orientation_;            ///< Measurement orientation subscriber
  //TODO: Acceleration not used yet 
  //ros::Subscriber sub_acceleration_;         ///< Measurement acceleration subscriber
  ros::Subscriber sub_reset_;                  ///< Reset Filter subscriber

	// @.@ Publishers
  ros::Publisher state_pub_;                   ///< State publisher
  ros::Publisher currents_pub_;                ///< Currents publisher
  ros::Publisher state_sensors_pub_;           ///< "State" with only information from the most recent sensors publisher
	
  // @.@ Services
  ros::ServiceServer set_vcurrent_velocity_srv_;
  ros::ServiceServer reset_vcurrent_srv_;

  // @.@ Timer
	ros::Timer timer_;                           ///< Principal timer iterator
  ros::Timer list_cleaner_timer_;              ///< Clear measurement list 
	tf2_ros::Buffer tfBuffer_;                   ///< Tf Buffer
  tf2_ros::TransformListener *tfListener_;     ///< tf listener

  // @.@ Frames tfs
  std::string world_frame_id_;                 ///< world frame name
  std::string base_frame_id_;                  ///< base frame name
  std::string map_frame_id_;                   ///< map frame name

	// @.@ Handy variables
  auv_msgs::NavigationStatus state_;           ///< State
	int zone_;                                   ///< GPS zone
	bool  northp_;                               ///< north hemisphere flag
	double origin_lat_;                          ///< latitude origin for tfs
  double origin_lon_;                          ///< longitude origin for tfs
  double origin_alt_;                          ///< altitude origib for tfs
	std::vector<FilterGimmicks::measurement> active_sensors_; ///< list of active sensors defined in yaml file
	bool  p_dvl_body_frame_;                     ///< Identify dvl frame

  std::string name_vehicle_id_;                ///< Name of the vehicle with the correspondent ID

	// @.@ Problem variables
	HorizontalFilter hFilter_;                   ///< Horizontal filter instantiation 
	VerticalFilter vFilter_;                     ///< Vertical filter instantiation
	RotationalFilter rFilter_;                   ///< Rotational filter instantiation

  // @.@ Virtual Currents Variables
  dsor_msgs::Measurement msg_vc_;
  bool vc_flag_{false};
  double vc_t_{0.0};
  double vc_vx_{0.0};
  double vc_vy_{0.0};
  double vc_offx_{0.0};
  double vc_offy_{0.0};
  
  double vc_last_t_{0.0};
  double vc_last_vx_{0.0};
  double vc_last_vy_{0.0};
  double vc_last_offx_{0.0};
  double vc_last_offy_{0.0};

  // @.@ Variables for "state" obtained from sensors [x,y,teta]
  double sensor_x_{0};
  double sensor_y_{0};
  double xy_time_{-1};
  double sensor_teta_{0};
  double teta_time_{-1};
  double sensor_vx_{0};
  double sensor_vy_{0};
  double v_time_{-1};
  farol_msgs::mState state_sensors_;

	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Initialize Subscribers 
   */
  /* -------------------------------------------------------------------------*/
  void initializeSubscribers();
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Initialize Publishers
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
   * @brief Initialize Timers 
   */
  /* -------------------------------------------------------------------------*/
  void initializeTimer();

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Load parameters from yaml file 
   */
  /* -------------------------------------------------------------------------*/
  void loadParams();

	// @.@ Callbacks declaration
	

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Main loop iteration of the sensor_fusion/filter  
   *
   * @param event
   */
  /* -------------------------------------------------------------------------*/
  void stateTimerCallback(const ros::TimerEvent &event);


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Clear the list of measurements in the horizontal filter after x seconds
   *
   * @param event
   */
  /* -------------------------------------------------------------------------*/
	void listTimerCallback(const ros::TimerEvent &event);
	

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Callback for receiving the measurements from sensors or simulations 
   *
   * @param msg Measurement type message, can be (position, vel, angles)
   *
   * @ note It is assumed that the measurements come always identified with position
   * vel or orientation
   */
  /* -------------------------------------------------------------------------*/
  void measurementCallback(const dsor_msgs::Measurement &msg);
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Resets the Filters
   *
   * @param msg
   */
  /* -------------------------------------------------------------------------*/
  void resetCallback(const std_msgs::Empty &msg);

	// @.@ Auxillary declarations
	

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Reads from the yaml file the sensors that will active affect the filter
   * outout
   *
   * @param valueXml parameters from yaml file
   *
   * @returns a vector of measurements, one for each defined sensor   
   */
  /* -------------------------------------------------------------------------*/
  std::vector<FilterGimmicks::measurement> readSensors(XmlRpc::XmlRpcValue valueXml);
	

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Reads from yaml the initial configuration of the filter, if no sensor(just
   * postion) is defined as the initial input.
   *
   * @param valueXml parameters from yaml file
   *
   * @returns returns a measurement for using in the filters  
   */
  /* -------------------------------------------------------------------------*/
  FilterGimmicks::measurement readManuallyInitialization(XmlRpc::XmlRpcValue valueXml);


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Helper method to read/parse parameters from yaml 
   *
   * @param double_vector
   *
   * @returns  vector of doubles with values from the yaml file
   */
  /* -------------------------------------------------------------------------*/
  std::vector<double> extractVectorDouble(XmlRpc::XmlRpcValue double_vector);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Helper method to read/parse parameters from yaml
   *
   * @param array 
   * @param double_array
   */
  /* -------------------------------------------------------------------------*/
  void extractArrayDouble(double* array, XmlRpc::XmlRpcValue double_array);
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Checks which variable state the sensor will affect (Hposition, Hvelocity, Vposition,orientation,acceleration, altitude). Then builds a corresponding horizontal, rotational or vertical message to send to the individual filters.  
   * 
   * @param m_in measurement from sensor or config file
   * @param m_horizontal Horizontal measurement (Hposition, Hvelocity, acceleration)
   * @param m_vertical Vertical measurement (Vposition/depth, altimeter)
   * @param m_rotation Rotational measurement (orientation)
   */
  /* -------------------------------------------------------------------------*/
  void sensorSplit(const FilterGimmicks::measurement &m_in, FilterGimmicks::measurement &m_horizontal, FilterGimmicks::measurement &m_vertical, FilterGimmicks::measurement &m_rotation);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Function to handle set_vcurrent_velocity service. Sets some relevant class variable 
   *
   * @param req service Request
   * @param res service Response
   */
  /* -------------------------------------------------------------------------*/
  bool setVCurrentVelocityService(sensor_fusion::SetVCurrentVelocity::Request &req, sensor_fusion::SetVCurrentVelocity::Response &res);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Function to handle reset_vcurrent service. Sets to 0 some relevant class variable 
   *
   * @param req service Request
   * @param res service Response
   */
  /* -------------------------------------------------------------------------*/
  bool resetVCurrentService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief This function simulates the effect of currents in the measurements from the sensors. It takes in a dsor_msgs Measurement message and changes it or not depending on the effect of the virtual currents active 
   *
   * @param msg the measurement to be changed
   */
  /* -------------------------------------------------------------------------*/
  void  virtualCurrents(const dsor_msgs::Measurement &msg);


};
#endif //CATKIN_WS_FILTERSNODE_H