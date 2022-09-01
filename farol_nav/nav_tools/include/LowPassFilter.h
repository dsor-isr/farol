//=================================================================================================
// Copyright (c) 2015, Farol Team, Instituto Superior Tecnico
// All rights reserved.
//=================================================================================================
// ROS Libs
#include <ros/ros.h>

/* -------------------------------------------------------------------------*/
/**
 * @brief  
 */
/* -------------------------------------------------------------------------*/
class LowPassFilter
{
public:
	/* Public Variables*/
	bool initialized;

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Constructor
   */
  /* -------------------------------------------------------------------------*/
    LowPassFilter(){
        ros::NodeHandle nh("~/insidepressure_filter/");

        nh.param("wn", wn, 0.01);               // filter bandwidth
        nh.param("qsi", qsi, 0.707);            // filter damping factor

        K1 = 2*qsi*wn;
        K2 = wn*wn;
        estimate.first = 0;
        estimate.second = 0;
        initialized = false;
        last_Update = ros::Time::now();
    }
    /* -------------------------------------------------------------------------*/
    /**
     * @brief  Starts initialized with 0's
     *
     * @Param wn
     * @Param qsi
     */
    /* -------------------------------------------------------------------------*/
	LowPassFilter(double wn, double qsi){
		K1 = 2*qsi*wn;
		K2 = wn*wn;
		estimate.first = 0;
		estimate.second = 0;
		initialized = true;		
		last_Update = ros::Time::now();
	}
	
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Can initialize on first measurement
   *
   * @Param wn
   * @Param qsi
   * @Param initialize_on_first
   */
  /* -------------------------------------------------------------------------*/
	LowPassFilter(double wn, double qsi, bool initialize_on_first){
		K1 = 2*qsi*wn;
		K2 = wn*wn;
		estimate.first = 0;
		estimate.second = 0;
		initialized = !initialize_on_first;
		last_Update = ros::Time::now();
	}

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Define the initial values to start
   *
   * @Param wn
   * @Param qsi
   * @Param var_init
   * @Param var_dot_init
   */
  /* -------------------------------------------------------------------------*/
	LowPassFilter(double wn, double qsi, double var_init, double var_dot_init){
		K1 = 2*qsi*wn;
		K2 = wn*wn;
		estimate.first = var_init;
		estimate.second = var_dot_init;
		initialized = true;
		last_Update = ros::Time::now();
	}

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
	~LowPassFilter(){
	}

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Param measurement
   *
   * @Returns   
   */
  /* -------------------------------------------------------------------------*/
	std::pair<double,double> update(double measurement){
		/* TODO: Robustify initialization */
		if(!initialized){
			estimate.first = measurement;
			estimate.second = 0;
			initialized = true;
			last_Update = ros::Time::now();
            return estimate;
		}
		double Ts = (ros::Time::now()-last_Update).toSec();

        /* Update estimates */
    	estimate.second = estimate.second + K2*(measurement - estimate.first);
        estimate.first = estimate.first + estimate.second*Ts + K1*(measurement - estimate.first);

        /* Save time of last update */
		last_Update = ros::Time::now();
        return estimate;
	}

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  
   *
   * @Returns   
   */
  /* -------------------------------------------------------------------------*/
	std::pair<double,double> predict(){
		if(!initialized)
            return estimate;

		double Ts = (ros::Time::now()-last_Update).toSec();
        std::pair<double, double> return_pair;

        /* Propagate assuming constant speed */
        return_pair.first = estimate.first + estimate.second*Ts;
        return_pair.second = estimate.second;

        return return_pair;

    }

protected:
	
	/* Gains */
	double wn, qsi;
	double K1, K2;

	/* Variables */
	std::pair<double,double> estimate; /* var and var_dot */
	ros::Time last_Update;
};
