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
		
		ros::Publisher  outlier_rejected_pub_, outlier_test_value_pub_, r_scale_pub_, k_pub_;
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
		
		
		bool usbl_outlier_rejection_{false};
		double outlier_threshold_;
		// --- Adaptive-R state (initialized in constructor) ---
		Eigen::Matrix3d R0_ = Eigen::Matrix3d::Identity();  // nominal measurement covariance (m^2)
		double r_scale_     = 1.0;     // adaptive scalar: R = r_scale_ * R0_
		double nis_target_  = 3.0;     // m = 3 (USBL is 3-DoF)
		double nis_beta_    = 0.2;    // adaptation rate (0.02..0.08 typical)
		double r_min_       = 0.0001;     // lower bound on scale
		double r_max_       = 1000.0;    // upper bound on scale
		double clip_c_      = 1.0;     // clip on log step to avoid jumps
		double hard_gate_   = 16.27;   // chi^2(3) 99.9% – hard reject safety net
		double p_floor_     = 2.5e-3;  // (0.05 m)^2 covariance floor per axis

		struct InputNode {
			double stamp;                 // input timestamp
			Eigen::Vector3d u;            // input velocity (Dock frame)
			Eigen::Vector3d x_snap;       // state after predicting to 'stamp'
			Eigen::Matrix3d P_snap;       // covariance after predicting to 'stamp'
		};

		struct Input {
			double stamp;
			Eigen::Vector3d u;
		};
		std::deque<Input> buf_;          // last ~2 s of inputs
		double window_sec_{2.0};         // keep a little margin

		// Sliding “front-of-window” snapshot (state at buf_.front().stamp)
		double snap_time_{-1.0};
		Eigen::Vector3d snap_x_ = Eigen::Vector3d::Zero();
		Eigen::Matrix3d snap_P_ = Eigen::Matrix3d::Identity();

		bool push_input_and_predict(const Stamped<Eigen::VectorXd>& meas);
		bool integrate_to(double t_target, Eigen::Vector3d& x,Eigen::Matrix3d& P,int& j,double& t);

		// Integrate (x,P,t) forward to 't_target' using piecewise-constant inputs in buf_,
		// starting from segment index 'j' (segment active at time 't').
		// On return: (x,P,t) advanced to min(t_target, last stamp), and 'j' points to the
		// active segment at the new time. Returns false if buf_ is empty or t_target < t.


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
		bool update(Stamped<Eigen::VectorXd> measurement, Eigen::Vector3d Z_D_in_B);

		// ROS stuff
		ros::NodeHandle nh_, nh_private_;
		ros::Publisher v1_B_pub_, v2_B_pub_, v1_D_pub_,v2_D_pub_, omega_1_pub_, omega_2_pub_;
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
		bool usbl_outlier_rejection_{false};
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
		inline double gate_LOS_on_S2(const Eigen::Vector3d& u_B_raw,
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

		bool push_input_and_predict(const Stamped<Eigen::VectorXd>& meas);
		bool integrate_to(double t_target, Sophus::SO3d& R, int& j, double& t, const Eigen::Vector3d& b);

  private:
		// ===== 2 s rolling window over gyro (AHRS rates) =====
		struct GyroInput {
			double stamp;
			Eigen::Vector3d w;  // body rates [rad/s]
		};
		std::deque<GyroInput> buf_;
		double window_sec_   = 2.0;

		// Snapshot (state at window start)
		double      snap_time_ = -1.0;
		Sophus::SO3d snap_R_;         // attitude at snap_time_
		Eigen::Vector3d snap_b_ = Eigen::Vector3d::Zero(); // bias at snap_time_

		// Bookkeeping for "present" prediction (optional but handy)
		double last_predict_time_ = -1.0;
		

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
		ros::Publisher usbl_pos_dock_pub_, usbl_pos_auv_pub_,usbl_yaw_pub_, terrain_normal_pub_, dvl_filt_pub_;
		geometry_msgs::Vector3 aux_vector3_msg_;
		Eigen::Vector3d aux_vec3_;
		std_msgs::Float64 float_aux_msg_;
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
		Eigen::Vector3d inertial_attitude_;
		Eigen::Vector3d Z_D_body_ = Eigen::Vector3d::UnitZ();
		Eigen::Vector3d dock_usbl_instalation_offset = Eigen::Vector3d::Zero();
		Eigen::Vector3d auv_usbl_instalation_offset = Eigen::Vector3d::Zero();

		bool dvl_outlier_rejection_{false};
		Stamped<Eigen::VectorXd> dvl_corrected_;
		

		struct DvlMiniKF {
			// State x = [v; a] in R^6  (v: m/s, a: m/s^2)
			bool initialized = false;
			double last_stamp = -1.0;

			Eigen::Matrix<double,6,1> x = Eigen::Matrix<double,6,1>::Zero();
			Eigen::Matrix<double,6,6> P = Eigen::Matrix<double,6,6>::Identity();

			// Noise (tunable)
			// Q models continuous-time acceleration random walk; discretized inside step()
			double q_acc = 0.5;                 // (m/s^2)^2 per second (process PSD for acceleration)
			Eigen::Matrix3d R = 0.04 * Eigen::Matrix3d::Identity(); // meas noise on velocity (m/s)^2

			// Constraints
			double a_max = 1.5;     // m/s^2  (max physical acceleration)
			double chi2_gate = 7.815;  // DoF=3, 95%

			// Telemetry
			double nis = 0.0;
			int rejected = 0;

			void configure(double q_acc_in, double r_meas, double amax, double chi2) {
				q_acc = q_acc_in;
				R = r_meas * Eigen::Matrix3d::Identity();
				a_max = amax;
				chi2_gate = chi2;
			}

			static inline void clamp_vec_norm(Eigen::Vector3d& v, double vmax) {
				double n = v.norm();
				if (n > vmax && vmax > 0.0) v *= (vmax / n);
			}
			static inline void clamp_vec_componentwise(Eigen::Vector3d& v, double vmax) {
				if (vmax <= 0.0) return;
				for (int i=0;i<3;++i) v[i] = std::clamp(v[i], -vmax, vmax);
			}

			// One step with constraints; returns true unless S not SPD (we keep prior if gated)
			bool step(const Stamped<Eigen::VectorXd>& z, Eigen::Vector3d& v_out) {
				const Eigen::Vector3d z_v = z.value.head<3>(); // measured velocity in Dock frame

				if (!initialized) {
					x << z_v, Eigen::Vector3d::Zero();
					last_stamp = z.stamp;
					initialized = true;
					v_out = x.head<3>();
					return true;
				}

				const double dt = std::max(0.0, z.stamp - last_stamp);
				last_stamp = z.stamp;

				// --- Predict: x+ = F x,  P+ = F P Fᵀ + Qd
				Eigen::Matrix<double,6,6> F = Eigen::Matrix<double,6,6>::Identity();
				// v_k+1 = v_k + dt * a_k
				F.block<3,3>(0,3) = dt * Eigen::Matrix3d::Identity();

				// Discretized Q for constant-accel with a random-walk acceleration PSD = q_acc
				// Qd = [ (1/3)dt^3 I, (1/2)dt^2 I; (1/2)dt^2 I, dt I ] * q_acc
				Eigen::Matrix<double,6,6> Qd = Eigen::Matrix<double,6,6>::Zero();
				Qd.block<3,3>(0,0) = (dt*dt*dt/3.0) * q_acc * Eigen::Matrix3d::Identity();
				Qd.block<3,3>(0,3) = (dt*dt/2.0)    * q_acc * Eigen::Matrix3d::Identity();
				Qd.block<3,3>(3,0) = (dt*dt/2.0)    * q_acc * Eigen::Matrix3d::Identity();
				Qd.block<3,3>(3,3) = dt             * q_acc * Eigen::Matrix3d::Identity();

				// Predict
				Eigen::Matrix<double,6,1> x_pred = F * x;
				Eigen::Matrix<double,6,6> P_pred = F * P * F.transpose() + Qd;

				// --- Innovation (H = [I3  0]) on velocity only
				Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero();
				H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

				Eigen::Vector3d nu = z_v - x_pred.head<3>();
				Eigen::Matrix3d S  = H * P_pred * H.transpose() + R;

				Eigen::LLT<Eigen::Matrix3d> llt(S);
				if (llt.info() != Eigen::Success) {
					// keep previous estimate, slightly inflate P to avoid sticking
					P += 1e-9 * Eigen::Matrix<double,6,6>::Identity();
					v_out = x.head<3>();
					return false;
				}

				nis = nu.transpose() * llt.solve(nu);
				if (nis > chi2_gate) {
					++rejected;
					// Reject measurement: keep prediction, but enforce acceleration constraint on predicted a
					x = x_pred;
					P = P_pred;
					// clamp acceleration magnitude
					Eigen::Vector3d a = x.tail<3>();
					clamp_vec_norm(a, a_max);
					x.tail<3>() = a;
					v_out = x.head<3>();
					return true;
				}

				// --- Update (Joseph form)
				Eigen::Matrix<double,6,3> K = P_pred * H.transpose() * llt.solve(Eigen::Matrix3d::Identity());
				Eigen::Matrix<double,6,1> x_new = x_pred + K * nu;

				const Eigen::Matrix<double,6,6> I6 = Eigen::Matrix<double,6,6>::Identity();
				Eigen::Matrix<double,6,6> P_new =
					(I6 - K*H) * P_pred * (I6 - K*H).transpose() + K * R * K.transpose();

				// --- Enforce physical constraints
				// 1) Clamp acceleration magnitude
				Eigen::Vector3d a_new = x_new.tail<3>();
				clamp_vec_norm(a_new, a_max);
				x_new.tail<3>() = a_new;

				// 2) Velocity slew-rate limit: |Δv| ≤ a_max * dt  (componentwise for simplicity)
				Eigen::Vector3d v_pred = x_pred.head<3>();
				Eigen::Vector3d v_new  = x_new.head<3>();
				Eigen::Vector3d dv     = v_new - v_pred;
				clamp_vec_componentwise(dv, a_max * dt);
				x_new.head<3>() = v_pred + dv;

				// Commit
				x = x_new;
				P = P_new;

				v_out = x.head<3>();
				return true;
			}
		};

	DvlMiniKF dvl_kf_;


};

