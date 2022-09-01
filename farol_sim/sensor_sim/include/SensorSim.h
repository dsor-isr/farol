/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico */
#ifndef CATKIN_WS_SENSORSIM_H
#define CATKIN_WS_SENSORSIM_H

// ROS Libraries
#include <ros/ros.h>

// ROS Messages and stuff
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <dsor_msgs/Measurement.h>
#include <dsor_msgs/Thruster.h>
#include <farol_msgs/mUSBLFix.h>
#include <farol_msgs/mThrusterStatus.h>
#include <auv_msgs/NavigationStatus.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/SetBool.h>

#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

// 3rd Parties
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <GeographicLib/UTMUPS.hpp>

#define RAD2DEG(x) x*180.0/FarolGimmicks::PI

class SensorSim
{
	typedef struct Sensor{
		
		enum Type {	null,
					AHRS,
					DVL_BT,
					DVL_WT,
					DEPTH,
					ALTIMETER,
					GNSS,
					RANGE,
					MODEL
		};
		
		Type type;
		std::string frame_id;
		bool debug;
		int count, thresh;
		double last_update;
		double frequency, variance;
		double noise[3], beacon[3];

		int zone; bool northp;
		double altitude;

		std::map<std::string, Type> enum_map;
		Sensor(){
			enum_map["null"] = null;
			enum_map["AHRS"] = AHRS;
			enum_map["DVL_BT"] = DVL_BT;
			enum_map["DVL_WT"] = DVL_WT;
			enum_map["DEPTH"] = DEPTH;
			enum_map["ALTIMETER"] = ALTIMETER;
			enum_map["GNSS"] = GNSS;
			enum_map["RANGE"] = RANGE;
			enum_map["MODEL"] = MODEL;
		}

	} Sensor;
public:
	// #############################
	// @.@ Constructor
	// #############################
	SensorSim(ros::NodeHandle *nh, ros::NodeHandle *nh_private);

	// #############################
	// @.@ Destructor
	// #############################
	~SensorSim();

	// #############################
	// @.@ Public methods
	// #############################
	double nodeFrequency();

private:
	ros::NodeHandle nh_, nh_private_;
  
  std::string name_vehicle_id_;                ///< Name of the vehicle with the correspondent ID
  std::string aux_frame_id_;
	
	// #####################
	// @.@ Subsctibers
	// #####################
	ros::Subscriber sub_odometry, sub_reset, sub_thruster;
	
	ros::Subscriber HSB_T, HPS_T, VSB_T, VPS_T;
	double HSB=0,HPS=0,VSB=0,VPS=0;
	ros::Time last_thrust_ref;	
	void HSB_Callback(const std_msgs::Int8& ptr);
	void HPS_Callback(const std_msgs::Int8& ptr);
	void VSB_Callback(const std_msgs::Int8& ptr);
	void VPS_Callback(const std_msgs::Int8& ptr);
    ros::Timer timer_thrust2;

    bool enable_dvl_{true};
    bool enable_altimeter_{true};
    ros::ServiceServer enable_dvl_srv_;
    ros::ServiceServer enable_altimeter_srv_;

	// #####################
	// @.@ Publishers
	// #####################
	ros::Publisher pub_position, pub_velocity, pub_orientation, pub_range, pub_model, pub_gnss;
	ros::Publisher pub_thrusters, pub_thrustStatus;
	
	// #######################
	// @.@ Timer
	// #######################
    ros::Timer timer_sensor;
    ros::Timer timer_thrust;

	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// CMakeLists.txt package.xml Parameters from Yaml
	double p_water_column_length;
	std::string world_frame_id;
	std::string base_link_frame_id; 

	// CMakeLists.txt package.xml Handy variables
	bool flag_reset, initialized;
	int SENSOR_COUNT;
	double origin_north, origin_east;
	ros::Time t_latest;

	// CMakeLists.txt package.xml Problem variables

    std::vector<Sensor> sensors;
	auv_msgs::NavigationStatus state;

	// #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void initializeSubscribers();
	void initializePublishers();
  void initializeServices();
	void initializeTimer();
	void loadParams();

	// #######################################################################################
	// @.@ Callbacks declaration
	// #######################################################################################
	void sensorTimerCallback(const ros::TimerEvent &event);
	void thrustTimerCallback(const ros::TimerEvent &event);
	void thrust2TimerCallback(const ros::TimerEvent &event);
	void stateCallback(const nav_msgs::Odometry &msg);
	void resetCallback(const std_msgs::Empty &msg);
	void thrustCallback(const dsor_msgs::Thruster &msg);
	void thrustStatusCallback(dsor_msgs::Thruster data);
    bool enableDVLService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool enableAltimeterService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);


	// #######################################################################################
	// @.@ Auxillary declarations
	// #######################################################################################
	bool addNoise(const auv_msgs::NavigationStatus &state, const Sensor &sensor, dsor_msgs::Measurement &m);
	bool addNoise(const auv_msgs::NavigationStatus &state, const Sensor &sensor, sensor_msgs::NavSatFix &m);
	void addNoise(const auv_msgs::NavigationStatus &state, const Sensor &sensor, farol_msgs::mUSBLFix &m);
	double randn(double mu, double sigma);

	std::vector<Sensor> extractSensors(XmlRpc::XmlRpcValue valueXml);

	void extractArrayDouble(double* array, XmlRpc::XmlRpcValue &double_array);
	template <typename T>
	void readXML(T &var, XmlRpc::XmlRpcValue &valueXml, T val){
		if (valueXml.getType() != XmlRpc::XmlRpcValue::TypeInvalid)
			var = static_cast<T> (valueXml);
		else
			var = val;
	}


};
#endif //CATKIN_WS_SENSORSIM_H
