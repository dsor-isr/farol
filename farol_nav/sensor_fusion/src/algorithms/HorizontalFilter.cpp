#include "HorizontalFilter.h"

HorizontalFilter::HorizontalFilter() {
  initialized_ = false;

  // +.+ Initialize All Eigen Matrices and Vectors
  reject_counter_.resize(MEAS_LEN);
  state_reject_counter_.resize(MEAS_LEN);
  state_reject_counter_.setZero();
  state_vec_.resize(STATE_LEN, 1);
  state_cov_.resize(STATE_LEN, STATE_LEN);
  state_vec_.setZero();
  state_cov_.setZero();
  process_cov_.resize(STATE_LEN, STATE_LEN);
  process_cov_.setZero();
}

bool HorizontalFilter::computePredict(auv_msgs::NavigationStatus &state, const ros::Time &t_request) {
	if (!initialized_)
    return false;

  predict(state_vec_, state_cov_, (t_request - last_predict_).toSec());
  last_predict_ = t_request;
  
	// +.+ Publish part transform from World (Horizontal) -> Horizontal
  geometry_msgs::TransformStamped world_to_base;
  world_to_base.header.stamp = t_request;
  world_to_base.header.frame_id = world_frame_id_;
  world_to_base.child_frame_id = world_frame_id_ + "_H";

  world_to_base.transform.translation.x = state_vec_(0);
  world_to_base.transform.translation.y = state_vec_(1);
  world_to_base.transform.rotation.w = 1;

  // +.+ Only broadcast if asked to
  if (tf_broadcast_flag_) {
    try {
      tf_broadcast_.sendTransform(world_to_base);
    } catch (tf2::TransformException &ex) {
      ROS_ERROR_DELAYED_THROTTLE(10.0, "Horizontal: %s", ex.what());
    }
  }

  // +.+ Return horizontal state estimate
  state.position.north = state_vec_(0);
  state.position.east = state_vec_(1);

  state.seafloor_velocity.x = state_vec_(2);
  state.seafloor_velocity.y = state_vec_(3);

  state.position_variance.north = state_cov_(0, 0);
  state.position_variance.east = state_cov_(1, 1);

  return true;
}

std::vector<double> HorizontalFilter::getExtimateCurrents(){
  // +.+ create a vector to retun the currents
  std::vector<double> currents;
  // +.+ add the currents to the vector  
  currents.emplace_back(state_vec_[6]);
  currents.emplace_back(state_vec_[7]);
  
  return currents;
}

void HorizontalFilter::configure(HorizontalFilter::config &configurations) {
  
	// +.+ Set TF parameterrs
  init_frame_id_ = configurations.meas_init.header.frame_id;
  world_frame_id_ = configurations.world_frame;
  name_vehicle_id_ = configurations.name_vehicle_id;

  tf_broadcast_ = *configurations.br_node;
  tf_broadcast_flag_ = configurations.broadcast_tf;

  // +.+ Set kalman filter parameterrs
  //t_period_ = configurations.kalman_config[0];
  t_save_measurement_ = configurations.kalman_config[1];
  t_reset_ = configurations.kalman_config[2];

  // +.+ Set initial state covariance and outlier rejection counter
  for (int i = 0; i < 6; i++)
    state_cov_(i, i) = configurations.meas_init.noise(i);

  for (int i = 0; i < 3; i++) {
    process_cov_.block<2, 2>(i * 2, i * 2) =
        Mat::Identity(2, 2) * configurations.process_noise[i];
    reject_counter_.segment<2>(i * 2) =
        Eigen::Vector2d(1, 1) * configurations.reject_counter[i];
  }
  
  // +.+ Parameters for currents are dependent on vehicle velocity because currents and vehicles velocity are derived from the same sensors
  state_cov_(6, 6) = configurations.meas_init.noise(2) * 0.001;
  state_cov_(7, 7) = configurations.meas_init.noise(3) * 0.001;
  process_cov_.block<2, 2>(6, 6) = Mat::Identity(2, 2) * configurations.process_noise[1] * 0.001;

  if (configurations.initialized)
    initialized_ = initialize(configurations.meas_init);
}

void HorizontalFilter::newMeasurement(FilterGimmicks::measurement &m) {
 
	// +.+ Return if the measurement is Invald
  if (FilterGimmicks::isinvalid(m, (last_update_.toSec() - t_save_measurement_))) {
    ROS_WARN("Horizontal: Measurement %s is invalid in the Horizontal Filter",
             m.header.frame_id.c_str());
    return;
  }
	
  // +.+ Initialize or update the filter state vector
  if (!initialized_ && m.header.frame_id == init_frame_id_) {
    initialized_ = initialize(m);
    ROS_WARN( "Horizontal: Initialized With %s Measurement: [%.2f,%.2f]", m.header.frame_id.c_str(), m.value(0), m.value(1));
  }else if (initialized_) {
      forwardPropagation(addMeasurementToBuffer(m));
   }
}

void HorizontalFilter::deleteMeasurementsInBuffer() {
  // +.+ Return if the filter is not initialized
  if (!initialized_)
    return;

  // +.+ Delete all but one measurements older than save_measuement_interval
  std::list<FilterGimmicks::measurement>::iterator it_measurement = meas_list_.begin();

  // +.+ If the list measurement has only one measurement is beacause we have clean all measurements  and we haven't recived any measurement, yet
  if (meas_list_.size() == 1){
    return;
  }

  if ((ros::Time::now() - (--meas_list_.end())->header.stamp).toSec() > t_save_measurement_){
    it_measurement = (--meas_list_.end());
    ROS_WARN("Horizontal: The last measurement is out of the buffer save period time - all list will be deleted");
  }

  // +.+ Starting from the first masurement, till the last measurement untill save
  // time period exceeds
  else{
    while (it_measurement != meas_list_.end() &&
         (ros::Time::now() - it_measurement->header.stamp).toSec() > t_save_measurement_) {
      it_measurement++;
    }
  }
  meas_list_.erase(meas_list_.begin(), it_measurement);
}

bool HorizontalFilter::initialize(FilterGimmicks::measurement &m) {
  // +.+ Set state vector and covariance if measurement is valid
  if (m.value.size() == MEAS_LEN) {
    state_vec_.segment<MEAS_LEN>(0) = m.value;
  } else if (m.value.size() == 2) {
    state_vec_.segment<2>(0) = m.value;
  } else {
    ROS_WARN("Horizontal: Could not initialize with measurement %s, passed "
             "value not initializable",
             m.header.frame_id.c_str());
    return false;
  }
  
  // +.+ Initialize eigen vectors and clear measurement list
  meas_list_.clear();
  m.state_copy.resize(STATE_LEN);
  m.state_cov_copy.resize(STATE_LEN, STATE_LEN);

  // +.+ Add measurement to measurement list
  m.state_copy = state_vec_;
  m.state_cov_copy = state_cov_;
  m.header.stamp = ros::Time::now();
  meas_list_.push_back(m);
  
  last_predict_ = ros::Time::now();
  last_update_ = ros::Time::now();

  return true;
}

void HorizontalFilter::predict(Vec &state_vec, Mat &state_cov, double dt) {
  // +.+ Return if prediction period is small
  if (dt < 0.001)
    return;

  // +.+ Make Kalman State matrix
  Mat state_mat = Mat::Identity(8, 8);
  // +.+ Add currents to the model
  state_mat.block<2, 2>(0, 2) << Mat::Identity(2, 2) * dt;
  //state_mat.block<2, 2>(0, 6) << Mat::Identity(2, 2) * dt;
	// TODO: Uncomment this to add the acceleratio ns to the model
	//state_mat.block<2, 2>(0, 4) << Mat::Identity(2, 2) * (dt * dt) * 0.5;
	//state_mat.block<2, 2>(2, 4) << Mat::Identity(2, 2) * dt;

  /* (NEW STATE) = (STATE TRANSITION MATRIX) * (PREV STATE)
   *
   *| X (k+1)  | = | 1 0 dt 0  0 0 0 0 |     | X  | position x
   *| Y (k+1)  | = | 0 1 0  dt 0 0 0 0 |     | Y  | position y
   *| vx (k+1) | = | 0 0 1  0  0 0 0 0  |     | vx | Inertial x velocity
   *| vy (k+1) | = | 0 0 0  1  0 0 0 0  |     | vy | Inertial y velocity
   *| ax (k+1) | = | 0 0 0  0  1 0 0 0  |  X  | ax | Acceleration x 
   *| ay (k+1) | = | 0 0 0  0  0 1 0 0  |     | ay | Acceleration y 
   *| cx (k+1) | = | 0 0 0  0  0 0 1 0  |     | cx | Current Velocity x 
   *| cy (k+1) | = | 0 0 0  0  0 0 0 1  |     | cy | Current Velocity y
   */

  // +.+ Propagate State and Covariance Matrix
  state_vec = state_mat * state_vec;
  state_cov = state_mat * state_cov * state_mat.transpose() + dt * process_cov_;
}

bool HorizontalFilter::update(Vec &state_vec, Mat &state_cov, std::list<FilterGimmicks::measurement>::iterator &it_measurement) {
 
  // +.+ kalman filter variables
  Vec if_vec;
  Mat observation_mat;
  Mat meas_cov;
  
  // +.+ Return if the filter is not initialized
  if (!initialized_)
    return false;
  last_update_ = it_measurement->header.stamp;

  // +.+ Find active sensor and create state observation matrix
  observation_mat.resize(2, STATE_LEN);
  observation_mat.setZero();
  meas_cov.resize(2, 2);
  meas_cov.setZero(2, 2);
  observation_mat = Eigen::MatrixXd::Zero(2, STATE_LEN);

  int temp = 0;
  for (int i = 0; i < it_measurement->config.size(); i++)
    if (it_measurement->config(i))
      observation_mat(temp++, i) = 1;

	// if_vec = Measurment - state mean
	// https://medium.com/blogyuxiglobal/kalman-filter-the-way-to-remove-outliers-bb6aa616788e
	if_vec = it_measurement->value - (observation_mat * state_vec);
  
  meas_cov.diagonal() = it_measurement->noise;

  // +.+ Calculate Kalman Innovation Matrix and Check if it is invertible
  Mat innovation_mat =
      observation_mat * state_cov * observation_mat.transpose() + meas_cov;
  Eigen::FullPivLU<Mat> lu(innovation_mat);
  if (!lu.isInvertible()) {
    ROS_ERROR(
        "Horizontal: Innovation Matrix <innovation_mat> is not invertible");
    return false;
  }
	// +.+ Mahalanobis distance
  //double normalized_error = if_vec.transpose() * innovation_mat.inverse() * if_vec;
  //std::cout << "Mahalanobis Distance " << normalized_error << "\n";

  // +.+ Outlier Rejection
  double dynamic_tolerance = it_measurement->outlier_tolerance + abs(it_measurement->header.stamp.toSec() - it_measurement->time_of_previous_meas) * it_measurement->outlier_increase;

	//if (normalized_error > it_measurement->outlier_tolerance)
  //if(if_vec.maxCoeff() > it_measurement->outlier_tolerance || if_vec.minCoeff() < -it_measurement->outlier_tolerance) 
  if(if_vec.maxCoeff() > dynamic_tolerance || if_vec.minCoeff() < -dynamic_tolerance) 
  {
    // +.+ Add one to all corresponding state reject counter
    state_reject_counter_ += it_measurement->config;
    // +.+ Check if any counter overflows
    if ((state_reject_counter_ - it_measurement->reject_counter*Eigen::VectorXd::Ones(MEAS_LEN)).maxCoeff() > 0){ 
      for (int i = 0; i < MEAS_LEN; i++){
        if (state_reject_counter_[i] > 0){
          state_reject_counter_[i] = 0;
        }
      }
    }else{
      //ROS_WARN("Horizontal: Measurement of frame %s rejected as outlier. x: %1f, y: %1f with normalized_error %1f and threshold %1f", it_measurement->header.frame_id.c_str(), if_vec(0), if_vec(1), normalized_error, outlier_rejection_threshold_);
      ROS_WARN("\nHorizontal: Measurement of frame %s rejected as outlier. Value: [%1f, %1f] with error of [%1f, %1f] and tolerance of %1f", it_measurement->header.frame_id.c_str(), it_measurement->value(0), it_measurement->value(1), if_vec(0), if_vec(1), it_measurement->outlier_tolerance);
      ROS_WARN("Horizontal Outlier Counter: POSITION: [x:%f, y:%f] , VELOCITY: [vx:%f, vy:%f]\n", state_reject_counter_(0), state_reject_counter_(1), state_reject_counter_(2), state_reject_counter_(3));
      return false;
    }
    
  }else{
    // +.+ If the outliers are not continous, reset state_reject_counter_ of the measurement received. Use measurement->config which tells the elements of the state affected. Note that if we have a config  [1 1 0 0 0 0 0 0], state_reject_counter_ is multiplied with [0 0 1 1 1 1 1 1 1] 
    Vec aux = (it_measurement->config.array()).select(it_measurement->config - Eigen::VectorXd::Ones(MEAS_LEN), Eigen::VectorXd::Ones(MEAS_LEN));
    Mat aux2 = aux.asDiagonal();
    state_reject_counter_ = aux2 * state_reject_counter_;
  }

  // +.+ Calculate Kalman gain matrix and update state vector and state covariance matrix
  Mat kalman_gain =
      state_cov * observation_mat.transpose() * innovation_mat.inverse();
  state_vec = state_vec + kalman_gain * if_vec;
  state_cov = (Mat::Identity(STATE_LEN, STATE_LEN) - kalman_gain * observation_mat) * state_cov;

  return true;
}

std::list<FilterGimmicks::measurement>::iterator
HorizontalFilter::addMeasurementToBuffer(FilterGimmicks::measurement &m) {
  // +.+ Return if measuement is Invalid
  if ((ros::Time::now() - m.header.stamp).toSec() > t_save_measurement_) {
    ROS_WARN("Horizontal: Trying to add measurement older than "
             "save_measurement_interval. Ignoring measurement.");
    return (--meas_list_.end());
  }

  // +.+ Insert current measurement in the time-ordered list
  std::list<FilterGimmicks::measurement>::iterator it_measurement = --meas_list_.end();
  
	// +.+ Find the element in the list that  has a timestamp lower than the measurement 
	while (it_measurement != meas_list_.begin() && it_measurement->header.stamp > m.header.stamp){
    it_measurement--;
  }

	// +.+ Advance the iterator one element in the list to place the new measurement in the right spot.
	// +.+ The iterator has to advance because the insert function will place the new measurement in the list before the element at the specified position.
  // +.+ check this: https://www.cplusplus.com/reference/list/list/insert/
  it_measurement = meas_list_.insert(++it_measurement, m);

  // +.+ This is done to point the iterator to the added measurement
  if (it_measurement != meas_list_.begin())
    return --it_measurement;
  else {
    ROS_ERROR("Horizontal: New measurement is the oldest in the list");
    return (--meas_list_.end());
  }
}

void HorizontalFilter::forwardPropagation(
    std::list<FilterGimmicks::measurement>::iterator it_measurement) {
  
  // +.+ Return if updating end measuement
  if (it_measurement == (--meas_list_.end())) {
    ROS_WARN("Horizontal: Trying to forward propagate from an end position "
             "iterator. This should happen only when initializing or ignoring "
             "old measurements.");
    return;
  }

  // +.+ Predict and update all measurements from the iterator position
  ros::Time t_estimate = it_measurement->header.stamp;
  state_vec_ = it_measurement->state_copy;
  state_cov_ = it_measurement->state_cov_copy;
  it_measurement++;
  while (it_measurement != meas_list_.end()) {
		predict(state_vec_, state_cov_, (it_measurement->header.stamp - t_estimate).toSec());
		t_estimate = it_measurement->header.stamp;
		// +.+ Update state vector and covariance if the update was valid (not rejected as outlier)
		if(update(state_vec_, state_cov_, it_measurement)){
			it_measurement->state_copy.resize(STATE_LEN);
			it_measurement->state_copy = state_vec_;
			it_measurement->state_cov_copy.resize(STATE_LEN, STATE_LEN);
			it_measurement->state_cov_copy = state_cov_;
			// +.+ update the iterator
			it_measurement++;
    }
    else{
      // +.+ Discard measurement 
      meas_list_.erase(it_measurement);
      t_estimate = (--meas_list_.end())->header.stamp;
      state_vec_ = (--meas_list_.end())->state_copy;
      state_cov_ = (--meas_list_.end())->state_cov_copy;
      break;
    }
  }
  // +.+ Predict until current time
  predict(state_vec_, state_cov_, (ros::Time::now() - t_estimate).toSec());
  last_predict_ = ros::Time::now();
}

void HorizontalFilter::resetFilter() {
  // +.+ Reset filter
  ROS_ERROR("Horizontal Filter reset.");
  initialized_ = false;
}