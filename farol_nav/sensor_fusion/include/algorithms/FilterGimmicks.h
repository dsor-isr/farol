/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico 

Don't you miss the danger
*/
#ifndef FILTERGIMMICKS_H
#define FILTERGIMMICKS_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <dsor_msgs/Measurement.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * @brief FilterGimmicks namespace
 * 
 * @note gimmicks methods for navigation filters
 */
namespace FilterGimmicks {


	/**
	 * @brief Define a measurement object
	 * 
	 * @note measurement struct temporary placed here. To be moved later somewhere
	 */
	typedef struct measurement {

		// CONFIG - from file, VALUE - from sensor
		enum measurement_type {Null = 1, CONFIG, VALUE};

		bool base_frame;
		std_msgs::Header header;
		Eigen::VectorXd value, noise, config, state_copy;
		Eigen::MatrixXd state_cov_copy;
		std::string sensor_config;
		double outlier_tolerance, time_of_previous_meas, outlier_increase;
		int reject_counter;

		void setLength(int length) {
			value.resize(length);
			noise.resize(length);
			config.resize(length);
		}

		// ########################################
		// Constructors for configurable, position, velocity and acceleration measurements
		// ########################################
			
		// Default Constructor
		measurement() {
			//base_frame = false;
		}

		measurement(int length) {
			//base_frame = false;
			value.resize(length);
			noise.resize(length);
			config.resize(length);
			value.setZero();
			noise.setZero();
			config.setZero();
			sensor_config = "";
			outlier_tolerance = 0.0;
			outlier_increase = 0.0;
			reject_counter = 0;
			time_of_previous_meas = 0.0;
		}

		measurement(std::string header_) {
			//base_frame = false;
			this->header.frame_id = header_;
		}

		// Used in extractSensors
		measurement(std::string frame_id, std::string config_, std::vector<double> noise_, double rejection_threshold_, int reject_counter_, double outlier_increase_){
		//base_frame = false;
		header.frame_id = frame_id;
		this->noise = Eigen::Map<Eigen::VectorXd> (noise_.data(), noise_.size());
		this->sensor_config = config_.data();
		this->outlier_tolerance = rejection_threshold_;
		this->reject_counter = reject_counter_;
		this->time_of_previous_meas = 0.0;
		this->outlier_increase = outlier_increase_;
    	}
  
		// Used in extractSensors
		measurement(std::string frame_id, std::vector<double> value_, std::vector<double> noise_) {
			//base_frame = false;
			header.frame_id = frame_id;
			this->noise = Eigen::Map<Eigen::VectorXd> (noise_.data(), noise_.size());
			this->value = Eigen::Map<Eigen::VectorXd> (value_.data(), value_.size());
		}

		
		// To convert ros data type to c++ data type -> see updateMeasurment
		measurement(const dsor_msgs::Measurement &m_):header(m_.header) {
			//base_frame = false;
			this->value = Eigen::Map<const Eigen::VectorXd> (m_.value.data(), m_.value.size());
			this->noise = Eigen::Map<const Eigen::VectorXd> (m_.noise.data(), m_.noise.size());
			this->outlier_tolerance = 0.0;
			this->reject_counter = 0;
			this->time_of_previous_meas = 0.0;
			this->outlier_increase = 0.0;
		}

	} measurement;


	/**
	 * @brief Predicate to find a giver frame_id among a list of frame_id
	 * 
	 */
	struct predicate_frame_id {
		std::string target; 
		predicate_frame_id(std::string target_) : target(target_){}
		bool operator()(measurement &sensor) { return sensor.header.frame_id == target;}
	};

  	/**
	 * @brief Validates a measurement if time is not negative, measurement or covariance are not nan.
	 * 
	 * @param val measurement
	 * @param time 
	 * @return true if measurement is invalid
	 * @return false if measurement is valid 
	 */
	static bool isinvalid(const measurement& m, double time){
    // +.+ Check if time is negative or zero
    if(m.header.stamp.toSec() < time)
        return true;

    // +.+ Check if any value is nan
    for (int i = 0; i < m.value.size(); i++)
        if(std::isnan(m.value(i)))
            return true;

    // +.+ Check if covariance is nan
    for (int i = 0; i < m.noise.size(); i++)
        if(std::isnan(m.noise(i)))
            return true;

    return false;

  }

};
#endif

