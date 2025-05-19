/** 
 *  @file   outer_loop.hpp
 *  @brief  Docking Controller Algorithm header file
 *  @author Ravi Regalo ravi.regalo@tecnico.ulisboa.pt Instituto Superior Tecnico
 *  @date   
*/
#pragma once

// Third Party Libraries
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <cmath> 
#include <deque>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <boost/lockfree/spsc_queue.hpp>
#include <optional>

#include <farol_docking/utils/docking_utils.hpp>  
#include <farol_docking/utils/median_utils.hpp>  
#include <farol_docking/utils/logging_utils.hpp>  


/**
 * @brief An outer-loop controller to achieve the docking task
 *
 * @note cenas
 */
class DockingController{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief  Contructor docking controller
         */
        DockingController();

        /**
         * @brief  Destructor docking controller
         */
        virtual ~DockingController() = default;
        
        /**
         * @brief  Configure parameters and shit
         */
        void configure(std::string param, double value);

        /**
         * @brief  Compute the controller's output
         */
        bool compute(double Dt);

        // Variables
        std::string phase_;

        // Vehicle States
        Eigen::Vector3d inertial_pos_;
        Eigen::Vector3d docking_pos_;
        Eigen::Vector3d body_vel_;
        Reference output_;
        Eigen::Vector3d current_estimate{0.0,0.0}; 
    
    private:

        /**
         * @brief  logic of changing states
         */
        void state_transition();

        /**
         * @brief  path following algorithm when going to the reference point
         */
        Eigen::VectorXd path_following(double Dt);
        
        // Path Following gains
        double K1_{2*0.7*0.1};
        double K2_{0.1*0.1};
        double Ka_{10}; 
        double sigma_{0.0};
        double cross_track_{0.0};
        double sigma_dot_{0.0};
        double yaw_correction_{0.0};
        double yaw_d_{0.0};
        double V_d_{0.0};
        double U_{0.0};

        

    private:
};

