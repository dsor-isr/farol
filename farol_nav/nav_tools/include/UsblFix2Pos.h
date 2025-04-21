/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico */
#ifndef CATKIN_WS_USBLFIX2POS_H
#define CATKIN_WS_USBLFIX2POS_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h> //ALWAYS need to include this

// ROS messages and stuff
#include <auv_msgs/NavigationStatus.h>
#include <farol_msgs/mUSBLFix.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/stateAcomms.h>
#include <dsor_msgs/Measurement.h>

// Farol Libraries and msgs
#include <farol_gimmicks_library/FarolGimmicks.h>

// Third Party Libraries
#include <GeographicLib/UTMUPS.hpp>

#define R 6378.1 //Earth radius

class UsblFix2Pos
{
public:
	// #############################
	// @.@ Constructor
	// #############################
	UsblFix2Pos(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	// #############################
	// @.@ Destructor
	// #############################
	~UsblFix2Pos();

private:
	ros::NodeHandle nh_, nh_private_;

	// #####################
	// @.@ Subsctibers
	// #####################
    ros::Subscriber sub_usbl, sub_state;

	// #####################
	// @.@ Publishers
	// #####################
	ros::Publisher pub_pose_fix, pub_usbl_est_state, pub_usbl_est_console, pub_usbl_est_console_auv0_, pub_usbl_est_console_auv1_;

	// timers
	ros::Timer list_cleaner_timer;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// +.+ package.xml Parameters from Yaml
	bool p_fix_type;
	int p_t_sync;
	double p_meas_noise;
	int auv0_source_id_;
	int auv1_source_id_;
	// +.+ Problem variables
	bool initialized;
	double state_stamp, state[3], state_var[2], state_lat_lon[2];
	std::list<farol_msgs::mUSBLFix> usblfix_list;
	std::list<farol_msgs::stateAcomms> stateAcomms_list;
  std::string name_vehicle_id_;

    // #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void initializeSubscribers();
	void initializePublishers();
	void initializeTimers();
	void loadParams();

	// #######################################################################################
	// @.@ Callbacks declarations
	// #######################################################################################
	/* 
	* Function Name: usblFixBroadcasterCallback
	* Inputs : usblFix message
	* Effects: publishes vehicle position, relative to the given anchor
	* Outputs: ..
	*/	
	void usblFixBroadcasterCallback(const farol_msgs::mUSBLFix &msg);
	/* 
	* Function Name: stateCallback
	* Inputs : State of the anchor
	* Effects: Updates anchor state
	* Outputs: ...
	*/	
	void stateCallback(const auv_msgs::NavigationStatus &msg);

  /* 
	* Function Name: stateAcommsCallback
	* Inputs : State of the anchor
	* Effects:
	* Outputs: ...
	*/	
  void stateAcommsCallback(const farol_msgs::stateAcomms &msg);

	// #######################################################################################
	// @.@ Supplimentary declarations
	// #######################################################################################
	/* 
	* Function Name: computePosition
	* Inputs : range, bearing, elevation
	* Effects: Converts the relative position from spherical co-ordinates to cartesian coordinagtes
	* Outputs: relative position in cartesian coordinates
	*/	
	void transformPosition(double spherical[3], double cartesian[3]);

  void getEstLatLon(double spherical[3], double latLon[2]);

	// --
	void listTimerCallback(const ros::TimerEvent &event);

};
#endif //CATKIN_WS_UsblFix2Pos_H
