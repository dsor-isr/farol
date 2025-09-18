/** 
 *  @file   docking_filter.hpp
 *  @brief  Docking Filter Algorithmic part header file
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
#include <chrono>

// ros libraries
#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>  
#include <geometry_msgs/Point.h>  
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <auv_msgs/NavigationStatus.h>
#include <auv_msgs/BodyForceRequest.h>
#include <dsor_msgs/Measurement.h>
#include <farol_msgs/mState.h>
#include <farol_msgs/mUSBLFix.h>

// farol libraries
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <farol_docking/utils/logging_utils.hpp>  
#include <farol_docking/utils/docking_utils.hpp>  
#include <farol_docking/utils/median_utils.hpp>  




/**
 * @brief   A Position filter in R³. Uses a complementary filter aproach, where 
 *          updates are made with position measurements as vectors in R³ and 
 *          predicts are made using velocity measurements in R³
 * 
 * @note bitches
 */
class PositionFilter{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief  Contructor Horizontal Filter
         */
        PositionFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private);

        /**
         * @brief  Destructor Horizontal Filter
         */
        virtual ~PositionFilter() = default;

        /**
         * @brief  Reset the filter to the initial position
         */
        void reset();
        
        /**
         * @brief  initialize the filter with an initial measurement
         */
        void initialize(Eigen::Vector3d initial_measurement);
        

        /**
         * @brief   Predict the state evolution based on the process 
         *          modeln and velocity measurements
         */
        bool predict(Stamped<Eigen::VectorXd> measurement);
        bool predict(double time);


        /**
         * @brief  Correct state estimate with a new position measurement
         *      Does also outlier rejection based on mahalanobis distance
         * @param[in] measurement A new position measurement in R³
         * @see Lekkas et al Mahalanobis outlier rejection
         */
        bool update(Stamped<Eigen::VectorXd> measurement);

        // ROS stuff
        ros::NodeHandle nh_, nh_private_;
        ros::Publisher usbl_pos_dock_pub_, usbl_pos_auv_pub_, terrain_normal_pub_;
        ros::Subscriber sub_Q_;
        ros::Subscriber sub_R_;
        geometry_msgs::Vector3 aux_vector3_msg_;
        Eigen::Vector3d aux_vec3_;
        
        ros::Publisher  outlier_rejected_pub_, outlier_test_value_pub_;
        std_msgs::Int8 int8_aux_msg_;
        std_msgs::Float64 float64_aux_msg_;


        
        // Kalman Filter variables
        Eigen::Vector3d state_;
        Eigen::Matrix3d state_cov_;
        Eigen::Vector3d initial_state_;
        Eigen::Matrix3d process_noise_;
        Eigen::Matrix3d measurement_noise_;
        
        Eigen::Vector3d innovation_vector_;
        Eigen::Matrix3d innovation_matrix_;
        Eigen::Matrix3d K_;
        
        // shit for the retroactive update
        Eigen::Vector3d state_at_last_update_;
        Eigen::Matrix3d state_cov_at_last_update_;
        double time_at_last_update_;
        double update_delay_;
        std::deque<Stamped<Eigen::VectorXd>> input_meas_buffer_;

        std::optional<Stamped<Eigen::VectorXd>> last_input_measurement_;
        double last_predict_time_{-1.0};
        
        
        bool usbl_outlier_rejection_{true};
        bool dvl_outlier_rejection_{true};
        double outlier_threshold_;
    private:
};


/**
 * @brief   A Position filter in SO(3). Uses a complementary filter aproach, where 
 *          updates are made with position measurements as matrices from the group
 *          SO(3) in R³ and predicts are made using velocity measurements in so(3),
 *          using the exponential map.
 * 
 * @note bitches
 */
class AttitudeFilter{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief  Contructor Attitude Filter
         */
        AttitudeFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private);


        /**
         * @brief  Desctructor Attitude Filter
         */
        virtual ~AttitudeFilter() = default;

        /**
         * @brief  Reset the filter
         */
        void reset();
        
        /**
         * @brief  initialize the filter with an initial measurement
         */
        void initialize(Sophus::SO3d initial_measurement);

        /**
         * @brief   Predict the state evolution based on the process 
         *          modeln and velocity measurements
         */
        void add_input_measurement(Stamped<Eigen::Vector3d> input_measurement);

        /**
         * @brief   Predict the state evolution based on the process 
         *          modeln and velocity measurements
         */
        bool predict(Stamped<Eigen::VectorXd> measurement);
        bool predict(double time);

        /**
         * @brief  Correct state estimate with a new position measurement
         */
        bool update(Stamped<Eigen::VectorXd> measurement, Eigen::Vector3d terrain_normal_body);

        // ROS stuff
        ros::NodeHandle nh_, nh_private_;
        ros::Publisher v1_B_pub_, v2_B_pub_, v1_D_pub_,v2_D_pub_;
        geometry_msgs::Vector3 aux_vector3_msg_;
        Eigen::Vector3d aux_vec3_;
        
        ros::Publisher  outlier_rejected_pub_, outlier_test_value_pub_;
        std_msgs::Int8 int8_aux_msg_;
        std_msgs::Float64 float64_aux_msg_;


  
        // Kalman Filter variables
        Sophus::SO3d state_;
        Eigen::Matrix3d state_cov_;
        Sophus::SO3d initial_state_;

        Eigen::Vector3d b_hat_;
        
        // Estimator gains
        double k1_, k2_, kp_, ki_;
        
        Eigen::Vector3d innovation_vector_;
        Eigen::Matrix3d innovation_matrix_;

        double mahalanobis_distance_;
        
        // shit for the retroactive update
        Sophus::SO3d state_at_last_update_;
        double time_at_last_update_;
        double update_delay_;
        std::deque<Stamped<Eigen::VectorXd>> input_meas_buffer_;
        
        // some other shit idk man 
        std::optional<Stamped<Eigen::VectorXd>> last_input_measurement_;
        double last_predict_time_{-1.0};
        

        bool usbl_outlier_rejection_{true};
        double outlier_threshold_;
        inline Eigen::Matrix3d projectorOnTangent(const Eigen::Vector3d& u_hat_unit) {
            return Eigen::Matrix3d::Identity() - u_hat_unit * u_hat_unit.transpose();
        }
        inline Eigen::Matrix3d pseudoInverseSym(const Eigen::Matrix3d& A, double eps = 1e-9) {
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d s = svd.singularValues(), s_inv = Eigen::Vector3d::Zero();
            for (int i = 0; i < 3; ++i) if (s[i] > eps) s_inv[i] = 1.0 / s[i];
            return svd.matrixV() * s_inv.asDiagonal() * svd.matrixU().transpose();
        }
        inline bool gate_LOS_on_S2(const Eigen::Vector3d& u_B_raw,
                                    const Eigen::Vector3d& u_D_raw,
                                    const Sophus::SO3d& R_BD,
                                    const Eigen::Matrix3d& Sigma_u) {
            const Eigen::Vector3d u_B  = u_B_raw.normalized();
            const Eigen::Vector3d u_D  = u_D_raw.normalized();
            const Eigen::Vector3d uhat = (R_BD.matrix().transpose() * u_D).normalized();
            const Eigen::Matrix3d Pi   = projectorOnTangent(uhat);
            const Eigen::Vector3d r    = Pi * (u_B - uhat);
            const Eigen::Matrix3d S    = Pi * Sigma_u * Pi;        // rank-2
            const double gamma         = r.transpose() * pseudoInverseSym(S) * r;
            return gamma;
        }

    private:
};



/**
 * @brief The Docking Filter Algorithm 
 * 
 * @note bitches
 */
class DockingFilter{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief  Contructor for the Filter Algorithm
         */
        
        DockingFilter(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private);

        /**
         * @brief  Destructor for the Filter Algorithm
         */
        ~DockingFilter();

        /**
         * @brief  Start measurement handler thread
         */
        void start(); 

        /**
         * @brief  Change the process and measurement noises, or the ubsl
         */
        void configure(std::string noise_type, double noise);
        void configure(std::string type, std::vector<std::string>);

        
        /**
         * @brief  initialize the filter with an initial measurement
         */
        void initialize(double stamp);

        /**
         * @brief  Reset the filter to the initial position
         */
        void reset();

        /**
         * @brief  Recovers the current state
         */
        Sophus::SE3d get_state();


        /**
         * @brief   Run to process every measurement once they are received by the filter structure.
         *          Waits on a conditional variable in order to acess a SPSC buffer. 
         *          This is where most of the action happens has measurements are processed whenever possible.
         *          UPDATE actions are handled here.
         *          PREDICT actions that are based on measurements are also handled here
         */
        void measurement_handler();

        /**
         * @brief   Predict the state evolution based on the process 
         *          model, aka last velocity 
         */
        bool predict(double time);
        

        /**
         * @brief  Extract the position and orientation from the set of usbl measurements 
         */
        Sophus::SE3d extract_se3(Sophus::Vector6d new_measurement);
  

        // ------------------------------------- Variables  ------------------------------------ //

        // ROS stuff
        ros::NodeHandle nh_, nh_private_;
        ros::Publisher usbl_pos_dock_pub_, usbl_pos_auv_pub_, terrain_normal_pub_, outlier_rejected_usbl_pos_pub_, outlier_rejected_usbl_att_pub_;
        geometry_msgs::Vector3 aux_vector3_msg_;
        Eigen::Vector3d aux_vec3_;
        Stamped<Eigen::VectorXd> aux_stamped_;
        

        // Filters
        std::unique_ptr<PositionFilter> position_filter_;
        std::unique_ptr<AttitudeFilter> attitude_filter_;

        // outlier rejection configuration
        std::vector<std::string> outlier_rejection_;
        
        // filter configurations
        bool initialized_{false};
        double t_last_predict_;     // Time of last predict
        double t_last_update_;      // Time of last update
        
        // buffer to store all incoming measurements
        boost::lockfree::spsc_queue<Measurement, boost::lockfree::capacity<16>> measurements_buffer_;
        std::mutex measurements_buffer_mutex_;
        std::condition_variable measurements_buffer_cond_var_;
        // thread to process all incoming messages
        std::thread measurement_handler_thread_;
        std::atomic<bool> running_{true};

        // initializer buffer
        std::vector<Eigen::VectorXd> initializer_buffer_; // really will be Vector6d, containing [auv(r,b,e), dock(r,b,e)]
        int initializer_size_{4};

        // Inertial attitudes
        Eigen::Vector3d auv_attitude_; // the attitude of the vehicle in inertial frame read by the ahrs
        Eigen::Vector3d dock_attitude_; // the attitude of the dock in the inertial frame, received over acoustics. only available if the dock has an AHRS 
        bool dock_has_ahrs_{false};

        // vector normal to the terrain in the inertial frame, used to know the relative orientation
        Eigen::Vector3d terrain_normal_ = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d dock_usbl_instalation_offset = Eigen::Vector3d::Zero();
        Eigen::Vector3d auv_usbl_instalation_offset = Eigen::Vector3d::Zero();

    private:
};

