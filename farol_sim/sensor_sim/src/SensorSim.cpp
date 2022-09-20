/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
#include "SensorSim.h"

/*
#######################################################################################################################
@.@ CONSTRUCTOR: put all dirty work of initializations here
Note the odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
#######################################################################################################################
*/
SensorSim::SensorSim(ros::NodeHandle *nh, ros::NodeHandle *nh_private):nh_(*nh), nh_private_(*nh_private){
  ROS_INFO("in class constructor of SensorSim");
  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();
}

/*
#######################################################################################################################
@.@ Destructor
#######################################################################################################################
*/
SensorSim::~SensorSim() {

  // +.+ shutdown publishers
  pub_gnss.shutdown();
  pub_range.shutdown();
  pub_model.shutdown();
  pub_position.shutdown();
  pub_velocity.shutdown();
  pub_orientation.shutdown();
  pub_thrusters.shutdown();
  pub_thrustStatus.shutdown();

  // +.+ shutdown subscribers
  sub_odometry.shutdown();
  sub_thruster.shutdown();

  // +.+ shutdown node
  nh_.shutdown();
}

/*
#######################################################################################################################
@.@ Member Helper function to set up subscribers;
note odd syntax: &SensorSim::subscriberCallback is a pointer to a member function of SensorSim
"this" keyword is required, to refer to the current instance of SensorSim
#######################################################################################################################
*/
void SensorSim::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for SensorSim");
  sub_odometry = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 1, &SensorSim::stateCallback , this); // 
}

/*
#######################################################################################################################
@.@ Member helper function to set up publishers;
#######################################################################################################################
*/
void SensorSim::initializePublishers() {
  ROS_INFO("Initializing Publishers for SensorSim");
  pub_position 	= nh_.advertise<dsor_msgs::Measurement>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/position", "position"), 10);
  pub_velocity 	= nh_.advertise<dsor_msgs::Measurement>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/velocity", "velocity"), 10);
  pub_orientation = nh_.advertise<dsor_msgs::Measurement>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/orientation", "orientation"), 10);
  pub_gnss  = nh_.advertise<sensor_msgs::NavSatFix>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/gnss", "gnss"), 10);
  pub_range = nh_.advertise<farol_msgs::mUSBLFix> (FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/range", "range"), 10);
  pub_model = nh_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "state"), 10);
  pub_thrustStatus = nh_.advertise<farol_msgs::mThrusterStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/thrustStatus", "thrustStatus"), 10);
}

void SensorSim::initializeServices(){

  enable_dvl_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, "services/enable_dvl", "enable_dvl"), &SensorSim::enableDVLService, this);

  enable_altimeter_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, "services/enable_altimeter", "enable_altimeter"), &SensorSim::enableAltimeterService, this);
}

/*
#######################################################################################################################
@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
#######################################################################################################################
*/
void SensorSim::initializeTimer() {
  timer_sensor = nh_.createTimer(ros::Duration(1.0/SensorSim::nodeFrequency()), &SensorSim::sensorTimerCallback, this);
}

/*
#######################################################################################################################
@.@ Set frequency of the node default is 10
#######################################################################################################################
*/
double SensorSim::nodeFrequency()
{
  double node_frequency;
  nh_.param("node_frequency", node_frequency, 10.0);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

/*
#######################################################################################################################
@.@ Load the parameters
#######################################################################################################################
*/
void SensorSim::loadParams() {
  ROS_INFO("Load the SensorSim parameters");

  p_water_column_length = FarolGimmicks::getParameters<double>(nh_private_, "water_column_length", 100);

  int zone; bool northp; double gamma, k;
  double origin_lat = FarolGimmicks::getParameters<double>(nh_private_, "originLat", 38.765852);
  double origin_lon = FarolGimmicks::getParameters<double>(nh_private_, "originLon", -9.09281873);
  try
  {
    GeographicLib::UTMUPS::Forward(origin_lat, origin_lon, zone, northp, origin_east, origin_north, gamma, k);
  }
  catch (const GeographicLib::GeographicErr::exception &ex)
  {
    ROS_WARN("SensorSim caught exception: %s", ex.what());
  }

  ROS_INFO("Loading the name of the vehicle with correspondent ID");
  name_vehicle_id_ = FarolGimmicks::getParameters<std::string>(nh_private_, "name_vehicle_id");

  // Get the frame id for the base_link and world frame
  world_frame_id = name_vehicle_id_ + '_' + FarolGimmicks::getParameters<std::string>(nh_, "world_frame");
  base_link_frame_id = name_vehicle_id_ + '_' + FarolGimmicks::getParameters<std::string>(nh_, "base_link");
  
  // ########################################
  // Input Sensors
  // ########################################
  sensors = extractSensors(FarolGimmicks::getParameters<XmlRpc::XmlRpcValue>(nh_private_, "sensors"));
  sensors[0].zone = zone;
  sensors[0].northp = northp;
  SENSOR_COUNT = sensors.size();

}

/*
#######################################################################################################################
@.@ Callbacks Section / Methods
#######################################################################################################################
*/

/*
#######################################################################################################################
@.@ Iteration via timer callback
#######################################################################################################################
*/
void SensorSim::sensorTimerCallback(const ros::TimerEvent &event) {
  // std::cout << "#1: " << initialized << std::endl;
  if (!initialized)
    return;
  for (int i = 0; i < SENSOR_COUNT; i++){
    // std::cout << "#2: " << sensors[i].count << " : " << sensors[i].thresh << std::endl;
    if (sensors[i].count++ > sensors[i].thresh && sensors[i].thresh != 0)
      continue;
    if ((ros::Time::now().toSec() - sensors[i].last_update) > 1/sensors[i].frequency){
      sensors[i].last_update = ros::Time::now().toSec();
      // TODO: Try using void pointer to make implementation more generic here
      if (sensors[i].type == Sensor::MODEL){
        pub_model.publish(state);
      }
      else if (sensors[i].type == Sensor::GNSS){
        sensor_msgs::NavSatFix m;
        sensors[i].zone = sensors[0].zone;
        sensors[i].northp = sensors[0].northp;
        if(addNoise(state, sensors[i], m));
        pub_gnss.publish(m);
      }
      else if (sensors[i].type == Sensor::RANGE){
        farol_msgs::mUSBLFix m;
        addNoise(state, sensors[i], m);
        pub_range.publish(m);
      }
      else{
        dsor_msgs::Measurement m;
        if(addNoise(state, sensors[i], m)){
          switch(sensors[i].type){
            case Sensor::AHRS:
              pub_orientation.publish(m);	break;
            case Sensor::DEPTH:
            case Sensor::ALTIMETER:
              pub_position.publish(m);	break;
            case Sensor::DVL_BT:
            case Sensor::DVL_WT:
              pub_velocity.publish(m);	break;
          }
        }
      }
    }
  }
}

/*
#######################################################################################################################
@.@ Callback Flag
#######################################################################################################################
*/
void SensorSim::stateCallback(const nav_msgs::Odometry &msg) {
  if(msg.header.stamp > t_latest){
    initialized = true;
    t_latest = msg.header.stamp;

    // convert and store odometry in world frame
    state.header.stamp = msg.header.stamp;
    state.header.frame_id = msg.header.frame_id;
    // transform position
    state.position.north = origin_north + msg.pose.pose.position.x;
    state.position.east  = origin_east  + msg.pose.pose.position.y;
    state.position.depth = msg.pose.pose.position.z;
    state.altitude 		 = p_water_column_length - state.position.depth;
    // Set linear speed - is transformed later
    state.body_velocity.x = msg.twist.twist.linear.x;
    state.body_velocity.y = msg.twist.twist.linear.y;
    state.body_velocity.z = msg.twist.twist.linear.z;
    try
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = msg.header.stamp;
      transformStamped.header.frame_id = world_frame_id;
      transformStamped.child_frame_id = base_link_frame_id;
      transformStamped.transform.rotation = msg.pose.pose.orientation;
      tf2::doTransform(state.body_velocity, state.seafloor_velocity, transformStamped);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_DELAYED_THROTTLE(10.0, "Could not transform sea_floor velocity: %s", ex.what());
      return;
    }

    // set angles
    tf2::Quaternion tf_quat;
    tf2::convert(msg.pose.pose.orientation, tf_quat);
    tf2::Matrix3x3 m3x3(tf_quat);
    m3x3.getRPY(state.orientation.x, state.orientation.y, state.orientation.z);
    state.orientation_rate = msg.twist.twist.angular;

    // set message status
    state.status = auv_msgs::NavigationStatus::STATUS_ALL_OK;
  }
}

void SensorSim::thrustStatusCallback(dsor_msgs::Thruster data){
  for (int i = 0; i < data.value.size(); i++){
    farol_msgs::mThrusterStatus thrustStatus;
    thrustStatus.header.stamp = ros::Time::now();
    thrustStatus.header.frame_id = std::to_string(i);
    thrustStatus.Speed = data.value[i];
    thrustStatus.Temperature = 20;
    pub_thrustStatus.publish(thrustStatus);
  }
}

bool SensorSim::enableDVLService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

  if(req.data){
    enable_dvl_ = true;
    res.success = true;
    res.message = "DVL activated";
  }
  else{
    enable_dvl_ = false;
    res.success = false;
    res.message = "DVL deactivated";
  }

  return true;
}

bool SensorSim::enableAltimeterService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

  if(req.data){
    enable_altimeter_ = true;
    res.success = true;
    res.message = "Altimeter activated";
  }
  else{
    enable_altimeter_ = false;
    res.success = false;
    res.message = "Altimter deactivated";
  }

  return true;
}

std::vector<SensorSim::Sensor> SensorSim::extractSensors(XmlRpc::XmlRpcValue valueXml){
  std::vector<Sensor> sensors;
  for (int32_t i = 0; i < valueXml.size(); ++i){
    Sensor sensor;

    readXML(aux_frame_id_,  valueXml[i]["frame_id"], std::string("0"));
    sensor.frame_id = name_vehicle_id_ + '_' + aux_frame_id_;
    readXML(sensor.debug, valueXml[i]["debug"], false);
    readXML(sensor.frequency, valueXml[i]["frequency"], 1.0);
    readXML(sensor.thresh, valueXml[i]["count"], 0);

    std::string type;
    readXML(type, valueXml[i]["type"], std::string("null"));
    sensor.type = sensor.enum_map[type];

    extractArrayDouble(sensor.noise, valueXml[i]["noise"]);
    readXML(sensor.variance, valueXml[i]["variance"], 0.0);
    readXML(sensor.altitude, valueXml[i]["altitude"], 0.0);

    extractArrayDouble(sensor.beacon, valueXml[i]["beacon"]);

    sensors.push_back(sensor);
  }
  return sensors;
}

bool SensorSim::addNoise(const auv_msgs::NavigationStatus &state, const Sensor &sensor, dsor_msgs::Measurement &m){
  m.header.stamp = ros::Time::now();
  m.header.frame_id = sensor.frame_id;

  double x;
  switch(sensor.type){
    case Sensor::AHRS:
      x = state.orientation.x + randn(0.0, sensor.noise[0]);	      	m.value.push_back(x);
      x = state.orientation.y + randn(0.0, sensor.noise[0]);	      	m.value.push_back(x);
      x = state.orientation.z + randn(0.0, sensor.noise[0]);	      	m.value.push_back(x);
      x = state.orientation_rate.x + randn(0.0, sensor.noise[1]);   	m.value.push_back(x);
      x = state.orientation_rate.y + randn(0.0, sensor.noise[1]);    	m.value.push_back(x);
      x = state.orientation_rate.z + randn(0.0, sensor.noise[1]);    	m.value.push_back(x);
      for (int i = 0; i < 3; i++)
        m.noise.push_back(sensor.noise[0] + sensor.variance);
      for (int i = 0; i < 3; i++)
        m.noise.push_back(sensor.noise[1] + sensor.variance);
      break;
    case Sensor::DVL_BT:
      if ((state.altitude > sensor.altitude && sensor.debug == false) || enable_dvl_ == false)
        return false;
      x = state.seafloor_velocity.x + randn(0.0, sensor.noise[0]);   	m.value.push_back(x);
      x = state.seafloor_velocity.y + randn(0.0, sensor.noise[0]);   	m.value.push_back(x);
      for (int i = 0; i < 2; i++)
        m.noise.push_back(sensor.noise[0] + sensor.variance);
      break;
    case Sensor::DVL_WT:
      if ((state.altitude < sensor.altitude && sensor.debug == false) || enable_dvl_ == false)
        return false;
      x = state.body_velocity.x + randn(0.0, sensor.noise[0]);   	m.value.push_back(x);
      x = state.body_velocity.y + randn(0.0, sensor.noise[0]);   	m.value.push_back(x);
      for (int i = 0; i < 2; i++)
        m.noise.push_back(sensor.noise[0] + sensor.variance);
      break;
    case Sensor::DEPTH:
      x = state.position.depth + randn(0.0, sensor.noise[0]);   	m.value.push_back(x);
      m.noise.push_back(sensor.noise[0] + sensor.variance);
      break;
    case Sensor::ALTIMETER:
      if (enable_altimeter_ == false) return false;
      x = state.altitude + randn(0.0, sensor.noise[0]);   		m.value.push_back(x);
      m.noise.push_back(sensor.noise[0] + sensor.variance);

      break;
  }

  return true;
}

bool SensorSim::addNoise(const auv_msgs::NavigationStatus &state, const Sensor &sensor, sensor_msgs::NavSatFix &m){
  if (state.position.depth > 0.5 && sensor.debug == false)
    return false;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = sensor.frame_id;

  double x = state.position.east + randn(0.0, sensor.noise[0]);
  double y = state.position.north + randn(0.0, sensor.noise[0]);
  try{
    GeographicLib::UTMUPS::Reverse(sensor.zone, sensor.northp, x, y, m.latitude, m.longitude);
  }
  catch(...){
    ROS_ERROR_DELAYED_THROTTLE(10.0, "Could not convert UTM to GPS");
    return false;
  }
  m.position_covariance[0] = sensor.noise[0] + sensor.variance;
  m.position_covariance[4] = sensor.noise[0] + sensor.variance;
  return true;
}

void SensorSim::addNoise(const auv_msgs::NavigationStatus &state, const Sensor &sensor, farol_msgs::mUSBLFix &m){
  m.header.stamp = ros::Time::now();
  m.header.frame_id = sensor.frame_id;

  double x = pow((state.position.north - sensor.beacon[0]), 2) + pow((state.position.east - sensor.beacon[1]), 2) + pow((state.position.depth - sensor.beacon[2]), 2);
  m.range = pow (x, 0.5) + randn(0.0, sensor.noise[0]);
  m.position_covariance[0] = sensor.noise[0] + sensor.variance;
}

// from http://phoxis.org/2013/05/04/generating-random-numbers-from-normal-distribution-in-c/
double SensorSim::randn(double mu, double sigma){
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;
  if (call)
  {
    call = !call;
    return (mu + sigma * (double)X2);
  }
  do
  {
    U1 = -1 + ((double)rand() / RAND_MAX) * 2;
    U2 = -1 + ((double)rand() / RAND_MAX) * 2;
    W = pow(U1, 2) + pow(U2, 2);
  } while (W >= 1 || W == 0);

  mult = sqrt((-2 * log(W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;
  call = !call;
  return (mu + sigma * (double)X1);
}

void SensorSim::extractArrayDouble(double* array, XmlRpc::XmlRpcValue &double_array){
  if(double_array.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
    return;
  if(double_array.getType() == XmlRpc::XmlRpcValue::TypeDouble){
    array[0] = static_cast<double>(double_array);
    return;
  }
  for (int32_t i = 0; i < double_array.size(); ++i) {
    if (double_array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      array[i] = (static_cast<double>(double_array[i]));
    else if(double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      array[i] = (static_cast<double>(static_cast<int>(double_array[i])));
  }
}

/*
#######################################################################################################################
@.@ Main
#######################################################################################################################
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "farol_sim_node"); //node name
  // +.+ create a node handle; need to pass this to the class constructor
  ros::NodeHandle nh, nh_private("~");

  ROS_INFO("main: instantiating an object of type SensorSim");

  // +.+ instantiate an SensorSim class object and pass in pointer to nodehandle for constructor to use
  SensorSim farolSim(&nh, &nh_private);

  // +.+ Added to work with timer -> going into spin; let the callbacks do all the work
  ros::spin();

  return 0;
}

