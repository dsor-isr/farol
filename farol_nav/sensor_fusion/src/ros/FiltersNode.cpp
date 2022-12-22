/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
*/

#include "FiltersNode.h"

FiltersNode::FiltersNode(ros::NodeHandle *nh, ros::NodeHandle *nh_private)
  : nh_(*nh), nh_private_(*nh_private) {
    ROS_INFO("in class constructor of FiltersNode");
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

    loadParams();
    initializeSubscribers();
    initializePublishers();
    initializeTimer();
  }

FiltersNode::~FiltersNode() {

  // +.+ shutdown publishers
  state_pub_.shutdown();
  currents_pub_.shutdown();

  // +.+ shutdown subscribers
  sub_reset_.shutdown();
  sub_position_.shutdown();
  sub_velocity_.shutdown();
  sub_orientation_.shutdown();
  //sub_acceleration_.shutdown();

  // +.+ stop timer
  list_cleaner_timer_.stop();
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

void FiltersNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for FiltersNode");
  sub_reset_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/reset", "reset"), 10, &FiltersNode::resetCallback, this);

  sub_position_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 10, &FiltersNode::measurementCallback, this);
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &FiltersNode::measurementCallback, this);
  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &FiltersNode::measurementCallback, this);
  }

void FiltersNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for FiltersNode");
  
  state_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "state"), 10);
  state_acoustic_pub_ = nh_private_.advertise<farol_msgs::stateAcomms>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state_acomms", "state_acomms"), 10);
  currents_pub_ = nh_private_.advertise<farol_msgs::Currents>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/currents", "currents"), 10);
}

void FiltersNode::initializeTimer() {
  list_cleaner_timer_ = nh_.createTimer(ros::Duration(10.0), &FiltersNode::listTimerCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0 / FiltersNode::nodeFrequency()), &FiltersNode::stateTimerCallback, this);
}

double FiltersNode::nodeFrequency() {
  double node_frequency;
  node_frequency = FarolGimmicks::getParameters<float>(nh_private_, "node_frequency", 10.0);
  nh_.param("node_frequency", node_frequency, 10.0);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

void FiltersNode::loadParams() {

  // +.+ Filters configurations variables
	HorizontalFilter::config p_hconfig;
	VerticalFilter::config p_vconfig;
	RotationalFilter::config p_rconfig;
  
  ROS_INFO("Loading the Node parameters");
  
  ROS_INFO("Loading the name of the vehicle with correspondent ID");
  name_vehicle_id_ = FarolGimmicks::getParameters<std::string>(nh_private_, "name_vehicle_id");
  
  vehicle_ID_ = FarolGimmicks::getParameters<int>(nh_private_, "vehicle_ID");
  
  p_hconfig.name_vehicle_id = name_vehicle_id_;
  p_vconfig.name_vehicle_id = name_vehicle_id_;
  p_rconfig.name_vehicle_id = name_vehicle_id_;

  // +.+ Load TF Parameters from the ros parameter server
  p_hconfig.broadcast_tf = FarolGimmicks::getParameters<bool>(nh_private_, "tf/broadcast", true);
  p_vconfig.broadcast_tf = p_hconfig.broadcast_tf;

  p_hconfig.base_frame = name_vehicle_id_ + '_' + FarolGimmicks::getParameters<std::string>(nh_private_, "tf/frames/base_link");
  p_hconfig.odom_frame = name_vehicle_id_ + '_' + FarolGimmicks::getParameters<std::string>(nh_private_, "tf/frames/odom");
  p_hconfig.map_frame = name_vehicle_id_ + '_' + FarolGimmicks::getParameters<std::string>(nh_private_, "tf/frames/map");
  p_hconfig.world_frame = name_vehicle_id_ + '_' + FarolGimmicks::getParameters<std::string>(nh_private_, "tf/frames/world");
  
  p_vconfig.base_frame = p_hconfig.base_frame;
  p_vconfig.odom_frame = p_hconfig.odom_frame;
  p_vconfig.map_frame = p_hconfig.map_frame;
  p_vconfig.world_frame = p_hconfig.world_frame;
  
  p_rconfig.base_frame = p_hconfig.base_frame;
  p_rconfig.odom_frame = p_hconfig.odom_frame;
  p_rconfig.map_frame = p_hconfig.map_frame;
  p_rconfig.world_frame = p_hconfig.world_frame;
  
  world_frame_id_ = p_hconfig.world_frame;
  map_frame_id_ = p_hconfig.map_frame;
  base_frame_id_ = p_hconfig.base_frame;

  static tf2_ros::TransformBroadcaster br_node;
  p_hconfig.br_node = &br_node;
  p_vconfig.br_node = &br_node;
  p_rconfig.br_node = &br_node;

  // +.+ Get Origin Global Position
  origin_lat_ = FarolGimmicks::getParameters<double>(nh_private_, "originLat",
      38.765852);
  origin_lon_ = FarolGimmicks::getParameters<double>(nh_private_, "originLon",
      -9.09281873);
  origin_alt_ =
    FarolGimmicks::getParameters<double>(nh_private_, "/originAlt", 1.0);

  double x, y, gamma, k;
  try {
    GeographicLib::UTMUPS::Forward(origin_lat_, origin_lon_, zone_, northp_, x, y,
        gamma, k);
  } catch (...) {
    ROS_ERROR("FiltersNode: Can not convert origin Lat/Lon, are they defined "
        "and valid?");
  }

  ROS_INFO("Loading the KF parameters");
  // +.+ Kalman Filter Parameters
  p_hconfig.kalman_config[0] = FarolGimmicks::getParameters<double>(nh_private_, "kalman_filter/predict_period", 0.1);
  p_hconfig.kalman_config[1] = FarolGimmicks::getParameters<double>(nh_private_, "kalman_filter/save_measurement_interval", 60);
  p_hconfig.kalman_config[2] = FarolGimmicks::getParameters<double>(nh_private_, "kalman_filter/reset_period", 300);
   
  memcpy(&p_vconfig.kalman_config, &p_hconfig.kalman_config,
      sizeof(p_hconfig.kalman_config));
  memcpy(&p_rconfig.kalman_config, &p_hconfig.kalman_config,
      sizeof(p_hconfig.kalman_config));

  double placeholder[6];
  // +.+ Process Covariance - Position, Velocity, Orientation, Orientation Rate, Acceleration and Altitude (6)
  extractArrayDouble(placeholder,
      FarolGimmicks::getParameters<XmlRpc::XmlRpcValue>(
        nh_private_, "kalman_filter/process_covariance"));
  {
    double process[] = {placeholder[1], placeholder[1], placeholder[4],
      placeholder[1]};
    memcpy(&p_hconfig.process_noise, &process, sizeof(process));
  }
  {
    double process[] = {placeholder[0], placeholder[1], placeholder[0]};
    memcpy(&p_vconfig.process_noise, &process, sizeof(process));
  }
  {
    double process[] = {placeholder[2], placeholder[3]};
    memcpy(&p_rconfig.process_noise, &process, sizeof(process));
  }

  // +.+ Initialization Parameters
  p_rconfig.bypass_ahrs = FarolGimmicks::getParameters<bool>(
      nh_private_, "kalman_filter/bypass_ahrs", true);

  FilterGimmicks::measurement m_initial = readManuallyInitialization(FarolGimmicks::getParameters<XmlRpc::XmlRpcValue>(nh_private_, "kalman_filter/manually_initialization"));
  m_initial.header.stamp = ros::Time::now();
  sensorSplit(m_initial, p_hconfig.meas_init, p_vconfig.meas_init, p_rconfig.meas_init);

  if (m_initial.header.frame_id == name_vehicle_id_ + "_" + "" || m_initial.header.frame_id == name_vehicle_id_ + "_" + "null"){
    p_hconfig.initialized = true;
    p_vconfig.initialized = true;
		p_rconfig.initialized = true;
    ROS_WARN("FILTER will Initialize With the Manually Initialization");
  }
  
  // +.+ Read dvl frame from config file
  p_dvl_body_frame_ = FarolGimmicks::getParameters<bool>(
      nh_private_, "dvl/body_frame", true);
  
  // +.+ Input Sensors
  ROS_INFO("Loading the Sensors parameters");
  active_sensors_ = readSensors(FarolGimmicks::getParameters<XmlRpc::XmlRpcValue>(
          nh_private_, "kalman_filter/sensors"));
  for (unsigned int i = 0; i < active_sensors_.size(); i++) {
    FilterGimmicks::measurement m_h, m_v, m_r;
    sensorSplit(active_sensors_[i], m_h, m_v, m_r);
    if (m_h.config.sum() > 0)
      p_hconfig.sensors.push_back(m_h);
    if (m_v.config.sum() > 0)
      p_vconfig.sensors.push_back(m_v);
    if (m_r.config.sum() > 0)
      p_rconfig.sensors.push_back(m_r);
  }
  // +.+ Load configuration parameters for Horz. Vert. and Rot. filters
  hFilter_.configure(p_hconfig);
  ROS_INFO("HorizontalFilter: Configured Successfully");
  
  vFilter_.configure(p_vconfig);
  ROS_INFO("VerticalFilter:   Configured Successfully");
  
  rFilter_.configure(p_rconfig);
  ROS_INFO("RotationalFilter: Configured Successfully");
}

// @.@ Callbacks Section / Methods
void FiltersNode::measurementCallback(const dsor_msgs::Measurement &msg) {
  FilterGimmicks::measurement m(msg);
  
  // +.+ Disregard input if measurement config does not match the value size
  std::vector<FilterGimmicks::measurement>::iterator it_active_sensor =
    std::find_if(std::begin(active_sensors_), std::end(active_sensors_),
        FilterGimmicks::predicate_frame_id(msg.header.frame_id));
  
  if (it_active_sensor == std::end(active_sensors_)) {
    //ROS_WARN("FilterNode: No active sensor for the mesurement received");
    return;
  } 
  else {
    m.base_frame = it_active_sensor->base_frame;
    m.sensor_config = it_active_sensor->sensor_config;
  }

  // +.+ Override with configure noise if it exists
  if (it_active_sensor->noise.sum() > 0){
    m.noise = it_active_sensor->noise;
    m.outlier_tolerance = it_active_sensor->outlier_tolerance;
    m.reject_counter = it_active_sensor->reject_counter;
    m.outlier_increase = it_active_sensor->outlier_increase;

    if (it_active_sensor->time_of_previous_meas == 0.0){
      it_active_sensor->time_of_previous_meas = (m.header.stamp).toSec();
      m.time_of_previous_meas = it_active_sensor->time_of_previous_meas;
    }
    else{
      m.time_of_previous_meas = it_active_sensor->time_of_previous_meas;
      it_active_sensor->time_of_previous_meas = (m.header.stamp).toSec();
    }
  }

  m.base_frame = false;
  FilterGimmicks::measurement m_h, m_v, m_r;
  sensorSplit(m, m_h, m_v, m_r);
  
  if(m.sensor_config == "Hposition" || m.sensor_config == "Hvelocity" || m.sensor_config == "acceleration"){
    hFilter_.newMeasurement(m_h);
  }
  else if(m.sensor_config == "Vposition" || m.sensor_config == "Vvelocity" || m.sensor_config == "altitude"){
    vFilter_.newMeasurement(m_v);
  }
  else if(m.sensor_config == "orientation"){
    rFilter_.newMeasurement(m_r);
  }
}

void FiltersNode::resetCallback(const std_msgs::Empty &msg) {
  hFilter_.resetFilter();
  vFilter_.resetFilter();
  rFilter_.resetFilter();
}

// @.@ Iteration via timer callback
void FiltersNode::stateTimerCallback(const ros::TimerEvent &event) {
  ros::Time tRequest = ros::Time::now();
  
	//TODO: why horizontal has a return when fails and the others not??
  if (!hFilter_.computePredict(state_, tRequest))
    return;

  std::vector<double> currents = hFilter_.getExtimateCurrents();

  //+.+ State of base_frame_id is estimated in world_frame_id at time tRequest
  rFilter_.computePredict(state_, tRequest);
  vFilter_.computePredict(state_, tRequest);

  state_.header.stamp = tRequest;
  state_.header.frame_id = base_frame_id_;

  state_.origin.latitude = origin_lat_;
  state_.origin.longitude = origin_lon_;
  state_.origin.altitude = origin_alt_;

  if (tfBuffer_.canTransform(world_frame_id_, base_frame_id_, ros::Time(0))) {
    geometry_msgs::TransformStamped transformStamped =
      tfBuffer_.lookupTransform(base_frame_id_, world_frame_id_, ros::Time(0));
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    tf2::doTransform(state_.seafloor_velocity, state_.body_velocity,
        transformStamped);
  } else {
    ROS_WARN_DELAYED_THROTTLE(
        10.0, "FilterNode: seafloor velocity is not transformable, is the "
        "right tf being published?");
  }

  // +.+ position in lat/lon
  if (world_frame_id_ == map_frame_id_) {
    try {
      double gamma, k;
      GeographicLib::UTMUPS::Reverse(zone_, northp_, state_.position.east,
          state_.position.north,
          state_.global_position.latitude,
          state_.global_position.longitude, gamma, k);
    } catch (...) {
      ROS_ERROR_DELAYED_THROTTLE(
          2.0, "FiltersNode: Could not convert from UTM to Lan/Lon");
    }
  }

  state_.status = auv_msgs::NavigationStatus::STATUS_ALL_OK;
  state_pub_.publish(state_);

  // +.+ create currents msg
  farol_msgs::Currents currents_msg;

  currents_msg.x_current = currents[0];
  currents_msg.y_current = currents[1];
  currents_msg.Header.stamp = ros::Time::now();
  // +.+ publish the corrents velocities
  currents_pub_.publish(currents_msg);

  // +-+ create state_acomms msg
  farol_msgs::stateAcomms state_acomms_msg;

  state_acomms_msg.header.stamp = ros::Time::now();
  state_acomms_msg.source_id = vehicle_ID_;
  state_acomms_msg.position.north = state_.position.north;
  state_acomms_msg.position.east = state_.position.east;
  state_acomms_msg.position.depth = state_.position.depth;
  state_acomms_msg.position_variance.north = state_.position_variance.north;
  state_acomms_msg.position_variance.east = state_.position_variance.east;
  state_acomms_msg.position_variance.depth = state_.position_variance.depth;
  state_acomms_msg.global_position.latitude = state_.global_position.latitude;
  state_acomms_msg.global_position.longitude = state_.global_position.longitude;
  state_acomms_msg.global_position.altitude = state_.global_position.altitude;

  state_acoustic_pub_.publish(state_acomms_msg);

}

void FiltersNode::listTimerCallback(const ros::TimerEvent &event) {
  hFilter_.deleteMeasurementsInBuffer();
}

// @.@ 
std::vector<FilterGimmicks::measurement>
FiltersNode::readSensors(XmlRpc::XmlRpcValue valueXml) {
  std::vector<FilterGimmicks::measurement> sensors;
  //TODO: It needs to be int32_t ???? why??
  for (int i = 0; i < valueXml.size(); ++i) {
    FilterGimmicks::measurement sensor(
        name_vehicle_id_ + '_' + static_cast<std::string>(valueXml[i]["frame_id"]),
        static_cast<std::string>(valueXml[i]["config"]),
        extractVectorDouble(valueXml[i]["noise"]),
        static_cast<double>(valueXml[i]["outlier_tolerance"]),
        static_cast<int>(valueXml[i]["reject_counter"]),
        static_cast<double>(valueXml[i]["outlier_increase"]));
    if (valueXml[i]["base_frame"].getType() ==
        XmlRpc::XmlRpcValue::TypeBoolean) {
      sensor.base_frame = static_cast<bool>(valueXml[i]["base_frame"]);
    }
    sensors.push_back(sensor);
  }
  return sensors;
}

// @.@ 
FilterGimmicks::measurement
FiltersNode::readManuallyInitialization(XmlRpc::XmlRpcValue valueXml) {
 // When manually we don't care about the name of the vehicle
  FilterGimmicks::measurement sensor(
      name_vehicle_id_ + '_' + static_cast<std::string>(valueXml["frame_id"]),
      extractVectorDouble(valueXml["value"]),
      extractVectorDouble(valueXml["noise"]));
  return sensor;
}

// @.@ 
std::vector<double>
FiltersNode::extractVectorDouble(XmlRpc::XmlRpcValue valueXml) {
  std::vector<double> vector;
  for (int i = 0; i < valueXml.size(); ++i) {
    switch (valueXml[i].getType()) {
      case XmlRpc::XmlRpcValue::TypeDouble:
        vector.push_back(static_cast<double>(valueXml[i]));
        break;
      case XmlRpc::XmlRpcValue::TypeInt:
        vector.push_back(static_cast<double>(static_cast<int>(valueXml[i])));
        break;
      // Do nothing in the case the type is invalid (cases that we do not handle)
      default:
        break;
    }
  }
  return vector;
}

// @.@
void FiltersNode::extractArrayDouble(double *array, XmlRpc::XmlRpcValue double_array) {
  
  for (int i = 0; i < double_array.size(); ++i) {
    if (double_array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      array[i] = (static_cast<double>(double_array[i]));
    else if (double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      array[i] = (static_cast<double>(static_cast<int>(double_array[i])));
  }
}

// @.@ 
void FiltersNode::sensorSplit(const FilterGimmicks::measurement &m_in,
    FilterGimmicks::measurement &m_horizontal,
    FilterGimmicks::measurement &m_vertical,
    FilterGimmicks::measurement &m_rotation) {
  // +.+ [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14]
  // +.+ [H H V H H V R R R R R  R  H  H  V]

  m_horizontal.setLength(HorizontalFilter::MEAS_LEN);
  m_vertical.setLength(VerticalFilter::MEAS_LEN);
  m_rotation.setLength(RotationalFilter::STATE_LEN);

  m_horizontal.noise.setZero();
  m_vertical.noise.setZero();
  m_rotation.noise.setZero();

  m_horizontal.config.setZero();
  m_vertical.config.setZero();
  m_rotation.config.setZero();

  // +.+ Splitting the Overhead data
  m_horizontal.base_frame = m_in.base_frame;
  m_horizontal.header.frame_id = m_in.header.frame_id;
  m_vertical.header.frame_id = m_in.header.frame_id;
  m_rotation.header.frame_id = m_in.header.frame_id;

  m_horizontal.header.stamp = m_in.header.stamp;
  m_vertical.header.stamp = m_in.header.stamp;
  m_rotation.header.stamp = m_in.header.stamp;
  
  m_horizontal.time_of_previous_meas = m_in.time_of_previous_meas;
   
  if (m_in.sensor_config == "Hposition"){
    // +.+ basically measurements from: gnss, usbl
    m_horizontal.config.segment<2>(0) = Eigen::VectorXd::Ones(2);
    m_horizontal.noise = m_in.noise;
    m_horizontal.outlier_tolerance = m_in.outlier_tolerance;
    m_horizontal.reject_counter = m_in.reject_counter;
    m_horizontal.outlier_increase = m_in.outlier_increase;
    if(m_in.value.size() > 0){
      m_horizontal.value = m_in.value; 
    }
  }
  else if (m_in.sensor_config == "Vposition"){
    // +.+ basically measurements from: depth
    m_vertical.config(0) = 1;
    m_vertical.noise = m_in.noise;
    m_vertical.outlier_tolerance = m_in.outlier_tolerance;
    m_vertical.reject_counter = m_in.reject_counter;
    m_vertical.outlier_increase = m_in.outlier_increase;
    if(m_in.value.size() > 0){
      m_vertical.value = m_in.value;
    }
  }
  else if (m_in.sensor_config == "Hvelocity"){
    // +.+ basically measurements from: dvl
    //TODO: add here the vz velocity - (to do in the next iteration of the code)
    m_horizontal.config.segment<2>(2) = Eigen::VectorXd::Ones(2);
    m_horizontal.noise = m_in.noise;
    m_horizontal.outlier_tolerance = m_in.outlier_tolerance;
    m_horizontal.reject_counter = m_in.reject_counter;
    m_horizontal.outlier_increase = m_in.outlier_increase;
    if(m_in.value.size() > 0){
      m_horizontal.value.resize(2);
      // +.+ if the measurements ara expressed in body frame
      if(p_dvl_body_frame_){
        // +.+ Convert velocities form the body to the inercial frame
        m_horizontal.value(0) = cos(DEG2RAD(state_.orientation.z))*m_in.value(0)  - sin(DEG2RAD(state_.orientation.z))*m_in.value(1);
        m_horizontal.value(1) = sin(DEG2RAD(state_.orientation.z))*m_in.value(0)  + cos(DEG2RAD(state_.orientation.z))*m_in.value(1);
        }else{
        // +.+ If in Inercial frame
        m_horizontal.value = m_in.value.segment<2>(0);  
      }
    }
  }
  else if (m_in.sensor_config == "Vvelocity"){
    // TODO: This is not being used - copy this "Hvelocity" to use the vz from the dvl
    m_vertical.config(1) = 1;
    m_vertical.noise = m_in.noise;
    m_vertical.outlier_tolerance = m_in.outlier_tolerance;
    m_vertical.reject_counter = m_in.reject_counter;
    m_vertical.outlier_increase = m_in.outlier_increase;
    if(m_in.value.size() > 0){
      m_vertical.value = m_in.value;
    }
  }
  else if (m_in.sensor_config == "orientation"){
    // +.+ basically measurements from: ahrs
    // +.+ angles
    m_rotation.config.segment<3>(0) = Eigen::VectorXd::Ones(3);
    m_rotation.noise.segment<3>(0) = m_in.noise.segment<3>(0);
    m_rotation.outlier_tolerance = m_in.outlier_tolerance;
    m_rotation.reject_counter = m_in.reject_counter;
    m_rotation.outlier_increase = m_in.outlier_increase;
    if(m_in.value.size() > 0){
      m_rotation.value.segment<3>(0) = m_in.value.segment<3>(0); 
    }
    // +.+ angle rates
    m_rotation.config.segment<3>(3) = Eigen::VectorXd::Ones(3);
    m_rotation.noise.segment<3>(3) = m_in.noise.segment<3>(3);
    m_rotation.outlier_tolerance = m_in.outlier_tolerance;
    m_rotation.reject_counter = m_in.reject_counter;
    m_rotation.outlier_increase = m_in.outlier_increase;
    if(m_in.value.size() > 0){
      m_rotation.value.segment<3>(3) = m_in.value.segment<3>(3); 
    }
  }
  else if (m_in.sensor_config == "acceleration"){
    // +.+ we dont have a on board sensor to measur that - is here for the FUTURE
    m_horizontal.config.segment<2>(4) = Eigen::VectorXd::Ones(2);
    m_horizontal.noise = m_in.noise;
    m_horizontal.outlier_tolerance = m_in.outlier_tolerance;
    m_horizontal.reject_counter = m_in.reject_counter;
    m_horizontal.outlier_increase = m_in.outlier_increase;
    if(m_in.value.size() > 0){
      m_horizontal.value = m_in.value; 
    }
  }
  else if(m_in.sensor_config == "altitude"){
    // +.+ basically measurements from: altimeter
    m_vertical.config(2) = 1;
    m_vertical.noise = m_in.noise;
    m_vertical.outlier_tolerance = m_in.outlier_tolerance;
    m_vertical.reject_counter = m_in.reject_counter;
    m_vertical.outlier_increase = m_in.outlier_increase;
    if(m_in.value.size() > 0){
      m_vertical.value = m_in.value; 
    }
  }
  else if(m_in.header.frame_id == name_vehicle_id_ + "_" + "" || m_in.header.frame_id == name_vehicle_id_ + '_' + "null"){
    // +.+ When we want to initialize manually
    m_horizontal.noise << m_in.noise.segment<2>(0), m_in.noise.segment<2>(3),
    m_in.noise.segment<2>(12);
    m_vertical.noise << m_in.noise(2), m_in.noise(5), m_in.noise(14);
    m_rotation.noise << m_in.noise.segment<6>(6);

    if (m_in.value.size() == 15) {
      m_horizontal.value << m_in.value.segment<2>(0), m_in.value.segment<2>(3),
      m_in.value.segment<2>(12);
      m_vertical.value << m_in.value(2), m_in.value(5), m_in.value(14);
      m_rotation.value << m_in.value.segment<6>(6);
    }
  }
}

// @.@ Main
int main(int argc, char **argv) {
  // +.+ ROS set-ups:
  ros::init(argc, argv, "filters_node"); // node name
  // +.+ create a node handle; need to pass this to the class constructor
  ros::NodeHandle nh, nh_private("~");

  ROS_INFO("main: instantiating an object of type FiltersNode");

  // +.+ instantiate an FiltersNode class object and pass in pointer to
  // nodehandle for constructor to use
  FiltersNode filters(&nh, &nh_private);
	
  // +.+ Dead reckoning
	DeadReckoning dr(&nh, &nh_private);
  // TODO: Added to work with timer -> going into spin; let the callbacks do all the work
  ros::spin();

  return 0;
}
