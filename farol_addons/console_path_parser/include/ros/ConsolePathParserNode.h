/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico

Don't you miss the danger ...
*/
#pragma once

// some generically useful stuff to include...
#include <math.h>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <list>
#include <filesystem>

// ROS Fundamentals
#include <ros/ros.h> 
#include <farol_gimmicks_library/FarolGimmicks.h>

// Package Related
#include <Section.h>
#include <Formation.h>

/* Required to call the path services*/
#include "dsor_paths/ResetPath.h"
#include "dsor_paths/SpawnArc2D.h"
#include "dsor_paths/SpawnLine.h"
#include "dsor_paths/SetConstSpeed.h"
#include "path_following/StartPF.h"
#include "path_following/StopPF.h"

// ROS messages and stuff
#include <auv_msgs/NavigationStatus.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <farol_msgs/Formation.h>
#include <farol_msgs/MultiSection.h>
#include <farol_msgs/Section.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#define PI 3.14159265
#define BUF_SIZE_TIME 80

/**
 * @brief Class Responsible for parsing a mission from the console Yebisu to farol_vx stack format
 * 
 */
class ConsolePathParserNode
{
public:
	/**
	 * @brief Construct a new Console Path Parser Node object
	 * 
	 * @param nodehandle 
	 * @param nodehandle_private 
	 */
	ConsolePathParserNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	/**
	 * @brief Destroy the Console Path Parser Node object
	 * 
	 */
	~ConsolePathParserNode();

	/**
	 * @brief 
	 * 
	 * @return double 
	 */
	double nodeFrequency();

	// +.+ Mission and Formation list
	std::list<Section> mission;
	std::list<Formation> formation;
	
	// +.+ Actual Section of the mission
	std::list<Section>::iterator act_section;
	
	// +.+ Without this variable gamma will be a huge value. Pfs will work, but not ideal
	farol_msgs::Section section_copy; 	
	
	ros::Time depth_end;

	// +.+ ID of the vehicle passed as parameter
	int own_id{0}; 

	// +.+ Section Variables
	double xrefpoint = 0;
	double yrefpoint = 0;
	double gamma_s = 0;
	double gamma_e = 0;
	double x_act = 0;
	double y_act = 0;
	double gamma = 0;
	double gamma_old = 0;
	double u_est = 0;

	// +.+ Formation Variables
	double x_forma = 0;
	double y_forma = 0;

	// +.+ Aux class variables
	float DesiredDepth = 0.0;
	bool wpOrient;
	bool ENABLE = false;
	bool formation_mode = false;
	bool biased_formation_mode = false;
	float node_frequency;
	std::string path_folder;

private:
	// +.+ Node handlers
	ros::NodeHandle nh_; 
	ros::NodeHandle nh_private_;

	// +.+ Subsctibers
	ros::Subscriber flag_sub_, missionstring_sub_, state_sub_;
	
	// +.+ Publishers
	ros::Publisher formation_pub_, biased_formation_pub_, section_pub_, wpref_pub_, fullpath_pub_, altitude_pub_, depth_pub_, flag_pub_;

  // +.+ Parameters
  bool p_console_new_{false};

	/* Path Service clients - used to construct the path to follow */
    ros::ServiceClient reset_path_client_;
    ros::ServiceClient spawn_arc_client_;
    ros::ServiceClient spawn_line_client_;
    ros::ServiceClient set_path_speed_client_;

    /* Path Following clients - to start and stop the path following algorithm */
    ros::ServiceClient stop_pf_client_;
    ros::ServiceClient start_pf_client_;

	// +.+ Timer
	ros::Timer timer_;

	// +.+ Reference position
	std_msgs::Float64 aux;
	geometry_msgs::PointStamped Reference_;

 	// #######################################################################################
 	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
 	// #######################################################################################
	 
	/**
	 * @brief Load parameters from ROS parameter server
	 * 
	 */
	void loadParams();
	
	/**
	 * @brief Initialize all node subscribers
	 * 
	 */
	void initializeSubscribers();
	
	/**
	 * @brief Initialize all node publishers
	 * 
	 */
	void initializePublishers();

	/**
	 * @brief Initialize all service publishers
	 * 
	 */
	void initializeServices();
	
	/**
	 * @brief Initialize node timer
	 * 
	 */
	void initializeTimer();

	// #######################################################################################
 	// @.@ Member helper functions of the class
 	// #######################################################################################s
	
	/**
	 * @brief Parsing string mission to the vehicle format of sections, used in missionStringCallback 
	 * 
	 * @param is 
	 */
	void parseMission(std::istream &is);
	
	/**
	 * @brief For cooperative pf, used in missionStringCallback
	 * 
	 */
	void missionFormation();
	
	/**
	 * @brief Manage switch between sections of the path from console, and what to do in the end
	 * 
	 */
	void startNewSection();
	
	/**
	 * @brief Wrap the angles
	 * 
	 * @param in 
	 * @return double 
	 */
	double ANG_P(double in)
	{
		return in < 0 ? 2 * PI + in : in;
	}

	/**
	 * @brief Method to make calls to the path service
	 * 
	 */
    void requestPath();

	// #######################################################################################
	// @.@ Callbacks declaration
	// #######################################################################################
	
	/**
	 * @brief 
	 * 
	 * @param event 
	 */
	void depthCallback(const ros::TimerEvent &event);
	
	/**
	 * @brief Accepts string from external source (Yebisu console) and tries to parse mission from it 
	 * 
	 * @param msg Mission String message 
	 */
	void missionStringCallback(const std_msgs::String &msg);
	
	/**
	 * @brief Check if we are doing a mission flag = 6, otherwise cancel 
	 * 
	 * @param msg Flag message 
	 */
	void flagCallback(const std_msgs::Int8 &msg);
	
	/**
	 * @brief 
	 * 
	 * @param msg  
	 */
	void updateCallback(const auv_msgs::NavigationStatus &msg);
	
	/**
	 * @brief Updates 2D position (X,Y) values in the State topic
	 * 
	 * @param msg Pose message
	 */
	void missionCallback(const std_msgs::String &msg);

	/**
	 * @brief Create directory for path mission text files from console
	 */
	void createPathFolder();
	
};

