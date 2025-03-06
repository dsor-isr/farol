#include "VerticalFilter.h"

VerticalFilter::VerticalFilter()
{
    // +.+ Flag indicating filter initialization
    initialized_ = false;
    init_with_depth_ = ros::Time::now();

    // +.+ Initialize All Eigen Matrices and Vectors
		reject_counter_.resize(MEAS_LEN);
    state_reject_counter_.resize(MEAS_LEN);
    state_reject_counter_.setZero();
    state_vec_.resize(MEAS_LEN);
    state_cov_.resize(MEAS_LEN, MEAS_LEN);
    process_cov_.resize(MEAS_LEN, MEAS_LEN);
    process_cov_.setZero();
    state_vec_.setZero();
    state_cov_.setZero();
		reject_counter_.setZero();

}

void VerticalFilter::configure(const VerticalFilter::config configurations)
{
    // +.+ Set TF parameters
    broadcast_tf_ = configurations.broadcast_tf;
    br_ = *configurations.br_node;
    world_frame_id_  = configurations.world_frame;
    name_vehicle_id_ = configurations.name_vehicle_id;

    // +.+ Set kalman filter parameters
    t_period_ = configurations.kalman_config[0];
    t_save_measurement_  = configurations.kalman_config[1];
    t_reset = configurations.kalman_config[2];

    // +.+ Set initial state covariance and outlier rejection counter
    for (int i = 0; i < 3; i++){
        process_cov_(i, i) = configurations.process_noise[i];
        reject_counter_(i) = configurations.reject_counter[i];
		}
		//process_cov_(3, 3) = 100;
    
		// +.+ Note: This will only run if you define a postion frame_id in config file.
		// Otherwise it only starts with a depth measurement.
    if (configurations.initialized){
			state_vec_.segment<3>(0)= configurations.meas_init.value;
			state_cov_.block<3,3>(0,0).diagonal() = configurations.meas_init.noise;
			last_predict_ = ros::Time::now();
			last_update_ = ros::Time::now();
			initialized_ = true;
		}

}

void VerticalFilter::computePredict(auv_msgs::NavigationStatus &state, ros::Time t_request)
{
    // +.+ Return if filter is not initialized
    if (!initialized_)
        return;

    // +.+ Reset the filter if t_request is much older than last_update
    if ((t_request - last_update_).toSec() > t_reset && t_reset != 0)
        resetFilter();
    
    predict(state_vec_, state_cov_, (t_request - last_predict_).toSec());
		last_predict_ = t_request;

    // +.+ Publish part transform from World (Vertical) -> Horizontal
    geometry_msgs::TransformStamped world_to_base;
    world_to_base.header.stamp = t_request;
    world_to_base.header.frame_id = world_frame_id_;
    world_to_base.child_frame_id = world_frame_id_ + "_V";
    world_to_base.transform.translation.z = state_vec_(0);
    world_to_base.transform.rotation.w = 1;

    if (broadcast_tf_)
    {
        try
        {
            br_.sendTransform(world_to_base);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR_DELAYED_THROTTLE(10.0, "Vertical: %s", ex.what());
        }
    }

    // +.+ Return vertical state estimate
    state.position.depth = state_vec_(0);
    state.altitude = state_vec_(2);
    state.global_position.altitude = state.altitude;
    state.seafloor_velocity.z = state_vec_(1);
    state.position_variance.depth = state_cov_(0,0);

}

void VerticalFilter::newMeasurement(const FilterGimmicks::measurement &m)
{
		if (FilterGimmicks::isinvalid(m, last_update_.toSec())){
        ROS_WARN("Vertical: Measurement %s is invalid in the Vertical Filter", m.header.frame_id.c_str());
        return;
    }
    // +.+ Initialize or update the filter state vector
    if (!initialized_)
    {
        //TODO: add a failed init msg in case
        ROS_WARN_THROTTLE(1.0, "Filter will initialize!!");
        initialized_ = initialize(m); // it will initialize close to this position
    }
    else{
        forwardPropagation(m);
    }
}

bool VerticalFilter::initialize(const FilterGimmicks::measurement &m)
{
    // +.+ Set state vector and covariance if measurement is valid
    state_vec_.setZero();
		// +.+ This checks if the measurement is from a depth sensor
    if(m.value.size() == 1 && m.config(0) == 1 && double((ros::Time::now() - init_with_depth_).toSec()) >= t_wait_altimeter_){
      state_vec_(0) = m.value(0);
			state_cov_(0,0) = m.noise(0);
      last_predict_ = ros::Time::now();
      last_update_ = ros::Time::now();
      ROS_WARN("Vertical: initialized with %s Measurement %f", m.header.frame_id.c_str(), m.value(0));
      return true;
     }
		// +.+ This checks if the measurement is from a altimeter sensor 
    if(m.value.size() == 1 && m.config(2) == 1){
      state_vec_(2) = m.value(0);
      state_cov_(2,2) = m.noise(0); 
			last_predict_ = ros::Time::now();
      last_update_ = ros::Time::now();
      ROS_WARN("Vertical: initialized with %s Measurement %.2f", m.header.frame_id.c_str(), m.value(0));
      return true;
     }
    return false;
}

void VerticalFilter::predict(Vec &state_vec, Mat &state_cov, double dt)
{
    // +.+ Return if prediction period is small
    if (dt < 0.001)
        return;

    // +.+ Make Kalman STATE TRANSITION MATRIX
    Mat state_mat = Mat::Identity(MEAS_LEN, MEAS_LEN);
    state_mat(0, 1) = dt;
    state_mat(2, 1) = -dt;

    // +.+ Make QUADRATIC DRAG TWEAK
    Vec process_vec = Vec::Zero(MEAS_LEN);

    /* (NEW STATE) = (STATE TRANSITION MATRIX) * (PREV STATE) + (INPUT MATRIX) * (INPUT) + PROCESS NOISE
    *
    *| d (k+1) | = | 1 dt  0 |   | d  | depth                | 
    *| vz (k+1)| = | 0 1   0 |   | vz | vertical velocity    | 
    *| a (k+1) | = | 0 -dt 1 | x | a  | altitude             |                
    */
    
    // +.+ Propagate State and Covariance Matrix
    state_vec = state_mat * state_vec; 
    state_cov = state_mat * state_cov * state_mat.transpose() + dt * process_cov_;
}

void VerticalFilter::update(Vec &state_vec, Mat &state_cov, const FilterGimmicks::measurement &m)
{
    // +.+ Return if the filter is not initialized_
    if (!initialized_)
        return;

    last_update_ = m.header.stamp;

    int INPUT_LEN = m.config.sum();
    
    // +.+ Find active sensor and built the STATE OBSERVATION MATRIX
    Mat observation_mat = Eigen::MatrixXd::Zero(INPUT_LEN, MEAS_LEN);{
        int temp = 0;
        for (int i = 0; i < m.config.size(); i++){
            if (m.config(i))
                observation_mat(temp++, i) = 1;
	}}

    // +.+ Calculate Kalman Innovation Matrix and Check if it is invertible
    Vec if_vec = Eigen::VectorXd::Zero(INPUT_LEN);
    if_vec = m.value - (observation_mat * state_vec);

    Mat meas_cov = Eigen::MatrixXd::Zero(INPUT_LEN, INPUT_LEN);
    meas_cov.diagonal() = m.noise;

    Mat innovation_mat = observation_mat * state_cov * observation_mat.transpose() + meas_cov;
    Eigen::FullPivLU<Mat> lu(innovation_mat);
    if (!lu.isInvertible())
    {
        ROS_ERROR("Vertical: Innovation Matrix <innovation_mat> is not invertible");
        return;
    }

		// +.+ Outlier rejection 
		//double normalized_error = if_vec.transpose() * innovation_mat.inverse() * if_vec;
		//if (normalized_error > m.outlier_tolerance)
    if(if_vec.maxCoeff() > m.outlier_tolerance || if_vec.minCoeff() < -m.outlier_tolerance) 
		{
			//// Add one to all corresponding state reject counter
			state_reject_counter_ += m.config;
				
			//// Check if any counter overflows
			if ((state_reject_counter_ - m.reject_counter*Eigen::VectorXd::Ones(MEAS_LEN)).maxCoeff() > 0){ 
				for (int i = 0; i < MEAS_LEN; i++){
					if (state_reject_counter_(i) > 0){
						state_reject_counter_(i) = 0;
					}
				}
			}
		else{
				//ROS_WARN("Vertical - Measurement of frame %s rejected as outlier. normalized_error %1f with total rejections: %1f", m.header.frame_id.c_str(), normalized_error, state_reject_counter_.sum());
        //ROS_WARN("\nVertical: Measurement of frame %s rejected as outlier. Value: [%1f] with error of [%1f] and tolerance of %1f", m.header.frame_id.c_str(), m.value(0), if_vec(0), m.outlier_tolerance);
        //ROS_WARN("Vertical Outlier Counter: POSITION: %f , ALTITUDE: %f\n", state_reject_counter_(0), state_reject_counter_(2));
				return;
			}
		
		}else{
		//// +.+ If the outliers are not continous, reset state_reject_counter_ of the measurement received. Use m.config which tells the elements of the state affected. Note that if we have a config  [1 1 0], state_reject_counter_ is multiplied with [0 0 1] 
		Vec aux = (m.config.array()).select(m.config - Eigen::VectorXd::Ones(MEAS_LEN), Eigen::VectorXd::Ones(MEAS_LEN));
		state_reject_counter_.dot(aux);
	}

    // +.+ Calculate Kalman gain matrix and update state vector and state covariance matrix
    Mat kalman_gain = state_cov * observation_mat.transpose() * innovation_mat.inverse();
    state_vec = state_vec + (kalman_gain * if_vec);
    state_cov = (Mat::Identity(MEAS_LEN, MEAS_LEN) - kalman_gain * observation_mat) * state_cov;
    
}

void VerticalFilter::forwardPropagation(const FilterGimmicks::measurement &m)
{
    // +.+ Return if measurement is 0.1 seconds older than the time os last predict
    if ((last_predict_ - m.header.stamp).toSec() > 0.1)
    {
        ROS_WARN("Vertical: Trying to update %f second old state in Vert with %s measurement.", (m.header.stamp - last_predict_).toSec(), m.header.frame_id.c_str());
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

void VerticalFilter::resetFilter()
{
    // +.+ Reset filter 
    ROS_ERROR("Vertical Filter reset.");
    initialized_ = false;
}
