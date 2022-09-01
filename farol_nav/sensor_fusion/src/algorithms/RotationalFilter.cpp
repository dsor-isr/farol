#include "RotationalFilter.h"

#define RAD2DEG(x) ((x) * ((180.0)) / (PI))
#define DEG2RAD(x) ((x) * ((PI) / (180.0)))
#define ROUND(x) std::floor((x * 1000) + .5) / 1000

RotationalFilter::RotationalFilter()
{
    initialized_ = false;

    // +.+ Initialize TF Listener
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  
    // +.+ Initialize All Eigen Matrices and Vectors
    reject_counter_.resize(STATE_LEN);
    state_reject_counter_.resize(STATE_LEN);
    state_reject_counter_.setZero();
    state_vec_.resize(STATE_LEN);
    state_cov_.resize(STATE_LEN, STATE_LEN);
    state_vec_.setZero();
    state_cov_.setZero();
    process_cov_.resize(STATE_LEN, STATE_LEN);
    process_cov_.setZero();
}

void RotationalFilter::computePredict(auv_msgs::NavigationStatus &state, const ros::Time &t_request)
{
    // +.+ Return if filter is not initialized
    if (!initialized_)
        return;

    // +.+ Predict until current time, if ahrs is a measurement and not a input
    if (ahrs_as_input_ == false){ 
        predict(state_vec_, state_cov_, (t_request - last_predict_).toSec());
        last_predict_ = t_request;
    }
    
    
    // +.+ Publish full transform from world frame -> base frame id
    geometry_msgs::TransformStamped world_to_base, world_to_base_t;

    world_to_base.header.stamp = t_request;
    world_to_base.header.frame_id = world_frame_id_;
    world_to_base.child_frame_id = base_frame_id_;

    // +.+ Retrieve part transform from World (Horizontal) -> World
    //try
    //{
    //    world_to_base_t = tf_buffer_.lookupTransform(world_frame_id_, world_frame_id_ + "_H", ros::Time(0));
    //    world_to_base.transform.translation.x = world_to_base_t.transform.translation.x;
    //    world_to_base.transform.translation.y = world_to_base_t.transform.translation.y;
    //}
    //catch (tf2::TransformException &ex)
    //{
    //    ROS_WARN_DELAYED_THROTTLE(5.0, "Rotation, %s: filter could not find horizontal TF", world_frame_id_.c_str());
    //}
    // +.+ Retrieve part transform from World (Vertical) -> World
    //try
    //{
    //    world_to_base_t = tf_buffer_.lookupTransform(world_frame_id_, world_frame_id_ + "_V", ros::Time(0));
    //    world_to_base.transform.translation.z = world_to_base_t.transform.translation.z;
    //}
  //catch (tf2::TransformException &ex)
    //{
    //    ROS_WARN_DELAYED_THROTTLE(5.0, "Rotation, %s: filter could not find vertical TF", world_frame_id_.c_str());
    //}

    // +.+ Set part transform from World (Rotational) -> World
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(state_vec_(0), state_vec_(1), state_vec_(2)); // orientation of the sensor
    world_to_base.transform.rotation = tf2::toMsg(quat_tf);

    // +.+ Find world frame
    if (world_frame_id_ == odom_frame_id_)
    {
        // +.+ publish tf directly
        try
        {
            tf_broadcast_.sendTransform(world_to_base);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_ERROR_DELAYED_THROTTLE(10.0, "Rotational: %s", ex.what());
        }
    }
    else if (world_frame_id_ == map_frame_id_)
    {
        // +.+ Publish direct map -> base_link tf if odom does not exist
        if (odom_frame_id_.compare(name_vehicle_id_ + std::string("_null")) == 0){
            tf_broadcast_.sendTransform(world_to_base);
        }
        // +.+ Calculate inverse TF the other way around, as explained here: https://github.com/cra-ros-pkg/robot_localization/blob/melodic-devel/src/ros_filter.cpp, Line 1934
        else{
            try
            {
                // +.+ Check if base -> odom tf can be found
                if (tf_buffer_.canTransform(base_frame_id_, odom_frame_id_, ros::Time(0)))
                {
                    tf2::Transform odom_to_base, map_to_odom, world_to_base_tf;
                    tf2::fromMsg(world_to_base.transform, world_to_base_tf);
                    tf2::fromMsg(tf_buffer_.lookupTransform(base_frame_id_, odom_frame_id_, ros::Time(0)).transform, odom_to_base);
                    map_to_odom.mult(world_to_base_tf, odom_to_base);
                    world_to_base.transform = tf2::toMsg(map_to_odom);
                    world_to_base.child_frame_id = odom_frame_id_;
                    tf_broadcast_.sendTransform(world_to_base);
                }
                // +.+ Tell user that filter is expecting odom -> base tf
                else{
                    ROS_ERROR_DELAYED_THROTTLE(10.0, "Rotational: Can not publish %s tranform, waiting for %s frame", map_frame_id_.c_str(), odom_frame_id_.c_str());
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR_DELAYED_THROTTLE(10.0, "Rotational: %s", ex.what());
            }
        }
    }
    
    // +.+ Return horizontal state estimate
    
    // +.+ Set Rotation
    state.orientation.x = RAD2DEG(state_vec_(0));
    state.orientation.y = RAD2DEG(state_vec_(1));
    state.orientation.z = RAD2DEG(state_vec_(2));
    
    // +.+ Set Rotational Velocity
    state.orientation_rate.x = RAD2DEG(state_vec_(3));
    state.orientation_rate.y = RAD2DEG(state_vec_(4));
    state.orientation_rate.z = RAD2DEG(state_vec_(5));
    
    // +.+ Covariance Matrix - 6x6 Row Major Matrix
    state.orientation_variance.x = state_cov_(0, 0);
    state.orientation_variance.y = state_cov_(1, 1);
    state.orientation_variance.z = state_cov_(2, 2);

}

void RotationalFilter::configure(const RotationalFilter::config configurations)
{
    // +.+ Set TF parameterrs
    base_frame_id_   = configurations.base_frame;
    odom_frame_id_   = configurations.odom_frame;
    map_frame_id_    = configurations.map_frame;
    world_frame_id_  = configurations.world_frame;
    name_vehicle_id_ = configurations.name_vehicle_id;
    tf_broadcast_ = *configurations.br_node;

    // +.+ Set kalman filter parameterrs
    t_period_ = configurations.kalman_config[0];
    t_save_measurement_ = configurations.kalman_config[1];
    t_reset_ = configurations.kalman_config[2];
    ahrs_as_input_ = configurations.bypass_ahrs;

    // +.+ c.process_noise' and c.reject_counter' length should be 2
    for (int i = 0; i < 2; i++){
        process_cov_.block<3,3>(3*i, 3*i) = Eigen::Matrix3d::Identity() * configurations.process_noise[i];
        reject_counter_.segment<3>(i*3) = 
          Eigen::Vector3d(1,1,1) * configurations.reject_counter[i];
    }

		// +.+ Note: This will only run if you define a postion frame_id in config file.
		// Otherwise it only starts with a depth measurement.
    if (configurations.initialized){
        initialized_ = initialize(configurations.meas_init);
		}
}

void RotationalFilter::newMeasurement(const FilterGimmicks::measurement &m)
{
    // +.+ Return if the measurement is Invald
    if (FilterGimmicks::isinvalid(m, last_update_.toSec())){
        ROS_WARN("Rotational: Measurement %s is invalid in the Rotational Filter", m.header.frame_id.c_str());
        return;
    }
    // +.+ Initialize or update the filter state vector
    if (!initialized_)
    {
        //TODO: add a failed init msg in case
        initialized_ = initialize(m); 				
        ROS_WARN("Rotational: Initialized With %s Measurement %.2f", m.header.frame_id.c_str(), RAD2DEG(m.value(2)));
    }
    else{
        if(ahrs_as_input_ == true){
            if (m.value.size() == STATE_LEN){
                for (int i = 0; i < STATE_LEN/2; i++){
                    state_vec_(i) = FarolGimmicks::wrap2pi(m.value(i), 0);
                    state_vec_(i+3) = m.value(i+3);
                }
                last_predict_ = m.header.stamp;
                return;
            }
            else{
                ROS_ERROR("Rotational: param bypass_ahrs is set true but ahrs input is not full length(6)");
						}
				}
        else{
					forwardPropagation(m);
				}
		}
    return;
}

bool RotationalFilter::initialize(const FilterGimmicks::measurement &m)
{
    // +.+ Set state vector and covariance if measurement is valid
    if (m.value.size() == STATE_LEN) {
        state_vec_ = m.value;
				state_cov_.diagonal() = m.noise;
        last_predict_ = ros::Time::now();
        last_update_ = ros::Time::now();
        return true;
    }
    else{
        ROS_WARN("Rotational: Could not initialize with measurement %s, passed value not initializable", m.header.frame_id.c_str());
        return false;
    }

}

void RotationalFilter::predict(Vec &state_vec, Mat &state_cov, double dt)
{
    // +.+ Return if prediction period is small
    if (dt < 0.001)
        return;

    // +.+ Make Kalman State matrix
    Mat state_mat = Mat::Identity(STATE_LEN, STATE_LEN);
    //state_mat.block<3, 3>(0, 3) << Mat::Identity(3, 3) * dt;

    /* 
    * (NEW STATE) = (STATE TRANSITION MATRIX) * (PREV STATE)
    *
    *| r  (k+1) | = | 1 0 0 0 0 0 |   | r  | roll    
    *| p  (k+1) | = | 0 1 0 0 0 0 |   | p  | pitch   
    *| y  (k+1) | = | 0 0 1 0 0 0 |   | y  | yaw
    *| vr (k+1) | = | 0 0 0 1 0 0 | x | vr | roll rate 
    *| vp (k+1) | = | 0 0 0 0 1 0 |   | vp | pitch rate
    *| vy (k+1) | = | 0 0 0 0 0 1 |   | vy | yaw rate   
    *,
    */

    // +.+ Propagate State and Covariance Matrix
    state_vec = state_mat * state_vec;
    for (int i = 0; i < 3; i++)
        state_vec(i) = FarolGimmicks::wrap2pi(state_vec(i), 0);

    state_cov = state_mat * state_cov * state_mat.transpose() + dt * process_cov_;

}

void RotationalFilter::update(Vec &state_vec, Mat &state_cov, const FilterGimmicks::measurement &m)
{
    // +.+ Return if the filter is not initialized
    if (!initialized_)
        return;

    last_update_ = m.header.stamp;
    int input_len = m.config.sum();
    
    // +.+ Find active states and create state observation matrix
    Mat observation_mat = Eigen::MatrixXd::Zero(input_len, STATE_LEN);{
        int temp = 0;
        for (int i = 0; i < m.config.size(); i++){
            if (m.config(i))
                observation_mat(temp++, i) = 1;
      }
    }

    // +.+ Calculate Kalman Innovation Matrix and Check if it is invertible
    Vec if_vec = Eigen::VectorXd::Zero(input_len);
    int k = 0;
    for (int i = 0; i < STATE_LEN; i++)
    {
        if (m.config(i))
        {
            if (i < STATE_LEN / 2)
            {
                // +.+ special case because angles are subracted to return the lowest value between them
                if_vec(k) = FarolGimmicks::angleDiff(m.value(k), state_vec(i));

            }
            else
                if_vec(k) = m.value(k) - state_vec(i);
            k++;
        }
    }

    Mat meas_cov = Eigen::MatrixXd::Zero(input_len, input_len);
    meas_cov.diagonal() = m.noise;

    Mat innovation_mat = observation_mat * state_cov * observation_mat.transpose() + meas_cov;
    Eigen::FullPivLU<Mat> lu(innovation_mat);
    if (!lu.isInvertible()){
        ROS_ERROR("Rotational: Innovation Matrix <innovation_mat> is not invertible");
        state_vec(0) = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    // +.+ Outlier rejection 
    //Vec normalized_error = if_vec.transpose() * innovation_mat.inverse() * if_vec;
		//if (normalized_error(0) > m.outlier_tolerance)
    if(if_vec.maxCoeff() > m.outlier_tolerance || if_vec.minCoeff() < -m.outlier_tolerance) 
    {
      // Add one to all corresponding state reject counter
      state_reject_counter_ += m.config;
        
      // Check if any counter overflows
      if ((state_reject_counter_ - m.reject_counter*Eigen::VectorXd::Ones(STATE_LEN)).maxCoeff() > 0){ 
        for (int i = 0; i < STATE_LEN/2; i++){
          if (state_reject_counter_[i] > 0){
            state_reject_counter_[i] = 0;
          }
        }
      }
      else{
        //ROS_WARN("Rotacional: Measurement of frame %s rejected as outlier. x: %1f, y: %1f with normalized_error %1f and threshold %1f", m.header.frame_id.c_str(), if_vec(0), if_vec(1), normalized_error(0), outlier_rejection_threshold_);
        ROS_WARN("Rotacional: Measurement of frame %s rejected as outlier", m.header.frame_id.c_str());
        return;
      }
    
    }else{
    // +.+ If the outliers are not continous, reset state_reject_counter_ of the measurement received. Use m.config which tells the elements of the state affected. Note that if we have a config  [1 1 0 0 0 0], state_reject_counter_ is multiplied with [0 0 1 1 1 1] 
    Vec aux = (m.config.array()).select(m.config - Eigen::VectorXd::Ones(STATE_LEN), Eigen::VectorXd::Ones(STATE_LEN));
    state_reject_counter_.dot(aux);
  }

  // +.+ Calculate Kalman gain matrix and update state vector and state covariance matrix
  Mat kalman_gain = state_cov * observation_mat.transpose() * innovation_mat.inverse();
  state_vec = state_vec + kalman_gain * if_vec;
  state_cov = (Mat::Identity(STATE_LEN, STATE_LEN) - kalman_gain * observation_mat) * state_cov;
  for (int i = 0; i < 3; i++){
      state_vec(i) = FarolGimmicks::wrap2pi(state_vec(i), 0);
      state_vec(i+3) = ROUND(state_vec(i+3));
  }

  return;
}

void RotationalFilter::forwardPropagation(const FilterGimmicks::measurement &m)
{
    // +.+ Return if measuement is Invalid
    if (!initialized_){  
        ROS_WARN("Rotational: Filter not yet initialized.");
        return;
    }
    // +.+ Return if measurement is 0.1 seconds older than the time of the last predict
    if ((last_predict_ - m.header.stamp).toSec() > 0.1)
    {
        ROS_WARN("Rotational: Trying to update %f second old state in Vert with %s measurement.", (m.header.stamp - last_predict_).toSec(), m.header.frame_id.c_str());
        return;
    }
    else if (m.header.stamp > last_predict_){
			// +.+ Predict the state from the time of last predict until the time of the measurement
			predict(state_vec_, state_cov_, (m.header.stamp - last_predict_).toSec());
			last_predict_ = m.header.stamp;
		}

    // +.+ Update the state with the current measurement
    update(state_vec_, state_cov_, m);
}

void RotationalFilter::resetFilter()
{
    // +.+ Reset Rotational Filter
    ROS_WARN("Rotational Filter reset.");
    initialized_ = false;
}

