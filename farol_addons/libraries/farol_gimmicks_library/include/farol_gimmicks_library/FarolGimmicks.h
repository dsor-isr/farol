/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico 

Don't you miss the danger
*/
#ifndef CATKIN_WS_FAROLGIMMICKSNODE_H
#define CATKIN_WS_FAROLGIMMICKSNODE_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * @brief FarolGimmicks namespace
 * 
 * @note why the code of templates is here -> because linkage problems see https://stackoverflow.com/a/1353981
 */
namespace FarolGimmicks{

	const double PI = 3.14159265;	///< PI value

	/**
	 * @brief Get the Parameters object
	 * 
	 * @tparam T the type of data of a desired parameter
	 * @param _nh ros nodehandle
	 * @param parameter_name string with paramenter name
	 * @return T parameter value
	 * 
	 * @note Option not considering default value, so the config file must have the parameter;
	 */
	template <typename T> T getParameters(ros::NodeHandle &_nh, std::string const &parameter_name){
		T parameter;
		
		if (!_nh.getParam(parameter_name, parameter)) {
			ROS_ERROR("No parameter [%s] shutting down", parameter_name.c_str());
			ros::shutdown();
		}
		
		// +.+ Note: this was giving problems with vectors
		_nh.getParam(parameter_name, parameter);

		return parameter;
	}

	/**
	 * @brief Get the Parameters object
	 * 
	 * @tparam T the type of data of a desired parameter
	 * @param _nh ros nodehandle
	 * @param parameter_name string with parameter name
	 * @param default_value default value of the parameter
	 * @return T parameter value
	 * 
	 *  @note Option considering default value. Even if the parameter doesn't exist in config file it is possible to use a default value.
	 */
	template <typename T> T getParameters(ros::NodeHandle &_nh, std::string const &parameter_name, T default_value){
		
		T parameter;
		
		if (!_nh.getParam(parameter_name, parameter)) {
			parameter = default_value;
		}
		else{
			_nh.getParam(parameter_name, parameter);
		}
		return parameter;
	}

	/**
	 * @brief Get the Parameters object
	 * 
	 * @tparam T the type of data of a desired parameter
	 * @param _nh  ros nodehandle 
	 * @param parameter_name string with parameter name
	 * @param default_value default value of the parameter
	 * @param delete_param boolean to delete or not the parameter from parameter server
	 * @return T parameter value
	 * 
	 * @note Option considering default value. Even if the parameter doesn't exist in config file it is possible to use a default value. Removes parameter from parameter server
	 */
	template <typename T> T getParameters(ros::NodeHandle &_nh, std::string const &parameter_name, T default_value, bool delete_param){
		
		T parameter;
		
		if (!_nh.getParam(parameter_name, parameter)) {
			parameter = default_value;
		}
		else{
			_nh.getParam(parameter_name, parameter);
		}

		// +.+ Delete the param if flag is set true
		if (delete_param){
			_nh.deleteParam(parameter_name);
		}

		return parameter;
	}

	/**
	 * @brief Convert from spherical to cartesian coordinates. Used mainly with usbl fixes
	 * 
	 * @param bearing  horizontal angle between the direction of an object and another object or between it and the true north direction in degrees. 
	 * @param elevation angle measured between the horizontal and the vehicle line of sight to the object
	 * @param range distance to the object
	 * @param out_pos_cart cartesian coordinates pointer
	 */
	void spherical_to_cartesian(double bearing, double elevation, double range, double *out_pos_cart);
	
	/**
	 * @brief Returns the sign of a double 
	 * 
	 * @param v double value
	 * @return int 1 if value is positive
	 * @return int 0 if value is 0
	 * @return int -1 if value is negative
	 */
	int signVal(double v);
	
	/**
	 * @brief Wraps angle between [0, 2PI]  or [-PI, PI]
	 * 
	 * @param theta angle in radians 
	 * @param mode 0 = Wrap from [0, 2*pi]; 1 = Wrap from [-pi, pi]
	 * @return double wraped angle
	 */
	double wrap2pi(double theta, const int mode);

	/**
	 * @brief Wrap angle between [0, 2PI] 
	 * 
	 * @param in angle in radians
	 * @return double wraped angle
	 */
	double wrapTo2pi(double in);

	/**
	 * @brief Method to calculate the diference between angles correctly even if they wrap between -pi and pi
	 * 
	 * @param a angle 1 in radians 
	 * @param b angle 2 in radians
	 * @return double 
	 */
	double angleDiff(double a, double b);

	/**
	 * @brief 
	 * 
	 * @tparam A value type
	 * @tparam B publisher type
	 * @param pub publisher
	 * @param value value
	 */
	template <typename A, typename B>
	void publishValue(ros::Publisher& pub, B& value) {
		A aux_;
		aux_.data = value;
		pub.publish(aux_);
	}

};

#endif //CATKIN_WS_FAROLGIMMICKSNODE_H
