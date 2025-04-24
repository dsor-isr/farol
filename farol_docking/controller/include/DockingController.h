/** 
 *  @file   DockingController.h 
 *  @brief  Horizontal Filter DSORLab 
 *  @author Ravi Regalo
 *  @date   
 ***********************************************/
#ifndef CATKIN_WS_DOCKINGCONTROLLER_H
#define CATKIN_WS_DOCKINGCONTROLLER_H

// @.@ Third Party Libraries
#include <ros/ros.h> 
#include <Eigen/Eigen>
#include <cmath> 
#include "docking_utils.h"


/* -------------------------------------------------------------------------*/
/**
 * @brief Relative Filter for docking 
 * @note cenas
 */
/* -------------------------------------------------------------------------*/
class DockingController{
    public:
        // @.@ Methods
        /* -------------------------------------------------------------------------*/
        /**
         * @brief  Contructor Horizontal Filter
         */
        /* -------------------------------------------------------------------------*/
        DockingController();

        /* -------------------------------------------------------------------------*/
        /**
         * @brief  Desctructor Horizontal Filter
         */
        /* -------------------------------------------------------------------------*/
        virtual ~DockingController() = default;
        
        
        void tune(std::vector<double> params);
        void update_state(double x, double y, double yaw, double z);
        void update_speed(double u);
        bool compute_update(double Dt);

        // @.@ Variables
        bool initialized_;
        double x_{0}, y_{0}, yaw_{0}, z_{0}, u_{0}, v_{0};
        int controller_state_{1};
        Eigen::Vector3d output_;
        double K1_{2*0.7*0.1};
        double K2_{0.1*0.1};
        double Ka_{10}; 
        double sigma_{0.0};
        bool fully_actuated_{false};

        double cross_track_{0.0};
        double sigma_dot_{0.0};
        double yaw_correction_{0.0};
        double yaw_d_{0.0};
        double V_d_{0.0};
        double U_{0.0};
        

    private:
};
#endif //CATKIN_WS_DOCKINGFILTER_H

