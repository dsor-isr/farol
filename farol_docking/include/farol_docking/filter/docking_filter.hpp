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

#include <farol_docking/utils/docking_utils.hpp>  
#include <farol_docking/utils/median_utils.hpp>  
#include <farol_docking/utils/logging_utils.hpp>  




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
        PositionFilter();

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
        bool update(Eigen::Vector3d measurement);
  
        // Kalman Filter variables
        Eigen::Vector3d state_;
        Eigen::Matrix3d state_cov_;
        Eigen::Vector3d initial_state_;
        Eigen::Matrix3d initial_state_cov_;
        Eigen::Matrix3d process_noise_;
        Eigen::Matrix3d measurement_noise_;
        
        Eigen::Vector3d innovation_vector_;
        Eigen::Matrix3d innovation_matrix_;
        Eigen::Matrix3d K_;

        double mahalanobis_distance_;
        double outlier_threshold_;

        std::optional<Stamped<Eigen::VectorXd>> last_input_measurement_;
        double last_predict_time_;

        bool output_outlier_rejection_{true};
        bool input_outlier_rejection_{true};
        int outlier_rejected_{0};

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
        AttitudeFilter();


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
        bool update(Sophus::SO3d measurement);
  
        // Kalman Filter variables
        Sophus::SO3d state_;
        Eigen::Matrix3d state_cov_;
        Sophus::SO3d initial_state_;
        Eigen::Matrix3d initial_state_cov_;
        
        Eigen::Matrix3d process_noise_;
        Eigen::Matrix3d measurement_noise_;
        
        Eigen::Vector3d innovation_vector_;
        Eigen::Matrix3d innovation_matrix_;

        double mahalanobis_distance_;
        double outlier_threshold_;

        std::optional<Stamped<Eigen::VectorXd>> last_input_measurement_;
        double last_predict_time_;

        bool output_outlier_rejection_{true};
        bool input_outlier_rejection_{true};
        int outlier_rejected_{0};
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
        DockingFilter();

        /**
         * @brief  Destructor for the Filter Algorithm
         */
        ~DockingFilter();

        /**
         * @brief  Change the process and measurement noises, or the ubsl
         */
        void configure(std::string noise_type, Eigen::MatrixXd noise);
        void configure(std::string type, std::vector<std::string>);

        
        /**
         * @brief  initialize the filter with an initial measurement
         */
        void initialize();

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
        // Filters
        PositionFilter position_filter_;
        AttitudeFilter attitude_filter_;

        // outlier rejection configuration
        std::vector<std::string> outlier_rejection_;
        double position_outlier_threshold_;
        double attitude_outlier_threshold_;
        
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
        long unsigned int initializer_size_{4};

        // Inertial attitudes
        Eigen::Vector3d auv_attitude_; // the attitude of the vehicle in inertial frame read by the ahrs
        Eigen::Vector3d dock_attitude_; // the attitude of the dock in the inertial frame, received over acoustics. only available if the dock has an AHRS 
        bool dock_has_ahrs_{false};
    private:
};

