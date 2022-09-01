#pragma once

#include <ros/ros.h>

/* Include Farol Gimmicks for parameter reading */
#include <farol_gimmicks_library/FarolGimmicks.h>

/* Cooperative Path Following library */
#include "CPFControl.h"
#include "EventTriggered.h"

/* My costume message in other packages */
#include <dsor_paths/PathData.h>

/* My costume message in this package */
#include <farol_msgs/CPFGamma.h>

/* ROS messages */
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

/* Include the generated services for the CPF node */
#include "cpf_control/StartStop.h"
#include "cpf_control/ChangeTopology.h"

/** 
 *  @brief     ROS node to actually do the Cooperative Control 
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class CpfNode {
  public:
    
    /**
     * @brief  The constructor for the Cooperative control law
     *
     * @param nh   The public nodehandle
     * @param nh_p The private nodehandle
     */
    CpfNode(ros::NodeHandle * nh, ros::NodeHandle * nh_p);

    /**
     * @brief Destructor for the CPF Node. Here all the subscribers, publishers
     * and timers are shutdown
     */
    ~CpfNode();

  private:

    /**
     * @brief Pointer to the CPF algorithm 
     */
    Eigen::MatrixXi adjency_matrix_;
    CPFControl * cooperative_;
    
    /** 
     * @brief Temporary variables for this vehicle 
     */
    double gamma_{0.0};
    double vd_{0.0};

    /**
     * @brief The ID of this vehicle 
     */
    unsigned int ID_{0};

    /**
     * @brief The sequence counter for the messages sent
     */
    unsigned int seq_{0};
 
    /**
     * @brief ROS node handles
     */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_; 

    /**
     * @brief ROS subscribers 
     */
    ros::Subscriber external_gamma_sub_;
    ros::Subscriber internal_gamma_sub_;

    /**
     * @brief ROS publishers 
     */
    ros::Publisher vc_pub_;
    ros::Publisher cpf_server_pub_;

    /**
     * @brief ROS services
     */
    ros::ServiceServer startCPF_srv_;
    ros::ServiceServer stopCPF_srv_;
    ros::ServiceServer change_topology_srv_;

    /**
     * @brief ROS timer 
     */
    ros::Timer timer_;

    /**
     * @brief ROS initializations */
    double nodeFrequency();
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void initializeTimer();
        
    /**
     * @brief Auxiliar method to instantiate the default CPF controller 
     */
    CPFControl * createEventTriggeredControl();
    
    /** 
     * @brief Auxiliar method to stop the cpf controller 
     */
    bool stop();
    
    /**
     * @brief Callback where all the logic is located 
     */
    void timerIterCallback(const ros::TimerEvent& event);

    /**
     * @brief Callback for receiving the external vehicle data 
     */
    void externalInfoCallback(const farol_msgs::CPFGamma& msg);

    /**
     * @brief Callback for receiving the data from this vehicle 
     */
    void internalInfoCallback(const dsor_paths::PathData &msg);
    
    /**
     * @brief Services callbacks
     */
    bool StartService(cpf_control::StartStop::Request &req, cpf_control::StartStop::Response &res);
    bool StopService(cpf_control::StartStop::Request &req, cpf_control::StartStop::Response &res);
    bool ChangeTopologyService(cpf_control::ChangeTopology::Request &req, cpf_control::ChangeTopology::Response &res);
};

