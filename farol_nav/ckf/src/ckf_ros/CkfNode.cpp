/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "CkfNode.h"
#include "CkfAlgorithm.h"

// @.@ Constructor
CkfNode::CkfNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

  // Initialize state and matrices
  std_msgs::Empty aux;
  resetCallback(aux);
  timer_.start();
}

// @.@ Destructor
CkfNode::~CkfNode() {

  // +.+ shutdown publishers
  state_pub_.shutdown();

  // +.+ shutdown subscribers
  sub_reset_.shutdown();
  sub_position_.shutdown();
  sub_velocity_.shutdown();
  sub_orientation_.shutdown();

  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void CkfNode::loadParams() {
  ROS_INFO("Load the CkfNode parameters");

  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);

  /*
  // create sensor list to retrieve configurations specified in yaml file
  std::string sensor_str = FarolGimmicks::getParameters<std::string>(nh_private_, "sensors/list", "gnss,usbl,dvl_bt,ahrs");

  std::string delimiter = ",";
  std::string token;

  for(int i = 0; sensor_str.size() > 0; i++){
    token = sensor_str.substr(0, sensor_str.find(","));
    sensor_list_.push_back(token);
    if(sensor_str.find(",") == std::string::npos) sensor_str = sensor_str.substr(sensor_str.find(",") + 1);
    else break;
  }

  // create configuration structure vector for sensors
  std::vector<struct sensor_config> sensor_configurations;
  
  // extracting information from yaml
  for(int i = 0; i < (int) sensor_list_.size(); i++){
    struct sensor_config sc;
    sc.config = FarolGimmicks::getParameters<std::string>(nh_private_, "sensors/" + sensor_list_[i] + "/config", "Hposition");
    sc.noise = FarolGimmicks::getParameters<double>(nh_private_, "sensors/" + sensor_list_[i] + "/noise", {1.0});
    sc.outlier_tolerance = FarolGimmicks::getParameters<double>(nh_private_, "sensors/" + sensor_list_[i] + "/outlier_tolerance", 1.0);
    sc.reject_counter = FarolGimmicks::getParameters<int>(nh_private_, "sensors/" + sensor_list_[i] + "/reject_counter", 10);
    sc.outlier_increase = FarolGimmicks::getParameters<double>(nh_private_, "sensors/" + sensor_list_[i] + "/outlier_increase", 1.0);
    sensor_configurations.push_back(sc);
  }*/

  // sensor configuration 
  // HARD CODED!!!! BEWARE
  for(int i = 0; i < (int) sensor_list_.size(); i++){
    struct sensor_config sc;
    sc.name = sensor_list_[i];
    if(sc.name == "gnss"){
      sc.config.push_back(0);       // position in x
      sc.config.push_back(1);       // position in y
      sc.noise << 0.001, 0.0,       // noise
                  0.0, 0.001;
      sc.outlier_tolerance = 0.2;
      sc.reject_counter = 8;
    }else if(sc.name == "usbl"){
      sc.config.push_back(0);       // position in x
      sc.config.push_back(1);       // position in y
      sc.noise << 1.0, 0.0,         // noise
                  0.0, 1.0;
      sc.outlier_tolerance = 1.0;
      sc.reject_counter = 10;
      sc.outlier_increase = 0.3;
    }else if(sc.name == "dvl_bt"){
      sc.config.push_back(0);         // velocity in x
      sc.config.push_back(1);         // velocity in y
      sc.noise << 0.0225, 0.0,
                  0.0, 0.0225;        // noise
      sc.outlier_tolerance = 0.2;
      sc.reject_counter = 200;
    }/*else if(sc.name == "ahrs"){
      sc.config.push_back(2);     // yaw
      sc.config.push_back(5);     // yaw_rate
      sc.noise.push_back(0.001);  // noise in yaw
      sc.noise.push_back(1.0);    // noise in yaw_rate
      sc.outlier_tolerance = 0.5;
      sc.reject_counter = 12;
    }*/
    sensors_.push_back(sc);
  }
  
  predict_cov_ << 1.0, 0.0, 0.0, 0.0,   // Set the intial predict covariance here
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0;

  // init_value_ << 4290822.1983094830,     // Position -         x
  //                491906.60293571133,     //                    y
  //                0.0000000000000000,     // Current velocity - vcx
  //                0.0000000000000000;     //                    vcy

  // state_ << init_value_;

  update_cov_ << 1.0, 0.0, 0.0, 0.0,   // Set the intial update covariance here
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0;

  process_ << 1.0, 0.0, 0.0, 0.0,    // Process matrix
              0.0, 1.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0;

  input_ << 1.0, 0.0,    // input matrix
            0.0, 1.0,
            0.0, 0.0,
            0.0, 0.0;
  
  process_cov_ << 0.1, 0.0, 0.0, 0.0,    // process covariance
                  0.0, 0.1, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0;

  Eigen::MatrixXd input_noise;
  struct sensor_config aux;

  for(int i = 0; i < (int) sensors_.size(); i++){
    if(sensors_[i].name == "dvl_bt"){
      aux = sensors_[i];
      break;
    }
  }
  input_noise << aux.noise(0, 0), 0.0, 0.0, 0.0,
                 0.0, aux.noise(1, 1), 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0;
            
  process_v << 0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0;

  complementary_cov_ = process_cov_ + input_noise;
}


// @.@ Member helper function to set up subscribers
void CkfNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for CkfNode");
  sub_reset_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/reset", "reset"), 10, &CkfNode::resetCallback, this);

  sub_position_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 10, &CkfNode::measurementCallback, this);
  // sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &CkfNode::measurementCallback, this);
  // sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &CkfNode::measurementCallback, this);
}


// @.@ Member helper function to set up publishers
void CkfNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for CkfNode");

  state_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "state"), 10);
}


// @.@ Member helper function to set up services
void CkfNode::initializeServices() {
  ROS_INFO("Initializing Services for CkfNode");

}


// @.@ Member helper function to set up the timer
void CkfNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / node_frequency_), &CkfNode::timerIterCallback, this);
}


void CkfNode::measurementCallback(const dsor_msgs::Measurement &msg) {
  // check the type of measurement, then save it
  measurements_.push_back(msg);
}

void CkfNode::resetCallback(const std_msgs::Empty &msg){
  predict_cov_ << 1.0, 0.0, 0.0, 0.0,   // Set the intial predict covariance here
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0;

  state_ << 4290822.1983094830,     // Position -         x
            491906.60293571133,     //                    y
            0.0000000000000000,     // Current velocity - vcx
            0.0000000000000000;     //                    vcy

  update_cov_ << 1.0, 0.0, 0.0, 0.0,   // Set the intial update covariance here
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0;
}

void CkfNode::predict(double delta_t){
  
  // get newest dvl  measurement
  dsor_msgs::Measurement input;
  input.header.stamp.sec = 0;

  std::string aux;
  for(int i = 0; i < (int) measurements_.size(); i++){
    aux = measurements_[i].header.frame_id;
    // Verify if dvl message and if measure is within range of the CKF cycle time interval
    if(aux.find("dvl_bt") != 0 && measurements_[i].header.stamp <= kf_time){
      // Save measurement if newest or discard it
      if(measurements_[i].header.stamp > input.header.stamp){
        input = measurements_[i];
        measurements_.erase(measurements_.begin() + i);
      }else{
        measurements_.erase(measurements_.begin() + i);
      }
    }
  }

  // prepare dvl input
  Eigen::VectorXd u;
  for(int i = 0; i < (int) input.value.size(); i++){
    u(i) = input.value[i];
  }

  // get dvl sensor
  struct sensor_config dvl_sensor;
  for(int i = 0; i < (int) sensors_.size(); i++){
    if(sensors_[i].name.find("dvl_bt") != 0){
      dvl_sensor = sensors_[i];
    }
  }

  // calculate new state and covariance matrix
  state_ = (process_ + process_v * delta_t) * state_ + u * delta_t;
  predict_cov_ = (process_ + process_v * delta_t) * update_cov_ * Eigen::Transpose(process_ + process_v * delta_t) + process_cov_ + dvl_sensor.noise;
}

void CkfNode::update(){
  
  // get measurements
  dsor_msgs::Measurement gnss, usbl;
  gnss.header.stamp.sec = 0;
  usbl.header.stamp.sec = 0;

  std::string aux;
  for(int i = 0; i < (int) measurements_.size(); i++){
    aux = measurements_[i].header.frame_id;
    // Verify if gnss or usbl message and if measure is within range of the CKF cycle time interval
    if(measurements_[i].header.stamp <= kf_time){
      if(aux.find("gnss") != 0){
        // Save measurement if newest or discard it
        if(measurements_[i].header.stamp > gnss.header.stamp){
          gnss = measurements_[i];
          measurements_.erase(measurements_.begin() + i);
        }else{
          measurements_.erase(measurements_.begin() + i);
        }
      }else if(aux.find("usbl") != 0){
        // Save measurement if newest or discard it
        if(measurements_[i].header.stamp > usbl.header.stamp){
          usbl = measurements_[i];
          measurements_.erase(measurements_.begin() + i);
        }else{
          measurements_.erase(measurements_.begin() + i);
        }
      }
    }
  }

  // get gnss and usbl sensor
  struct sensor_config gnss_sensor, usbl_sensor;
  for(int i = 0; i < (int) sensors_.size(); i++){
    if(sensors_[i].name.find("usbl") != 0){
      usbl_sensor = sensors_[i];
    }else if(sensors_[i].name.find("gnss") != 0){
      gnss_sensor = sensors_[i];
    }
  }

  // check if gnss, usbl or both
  Eigen::MatrixXd R, C, identity;
  Eigen::VectorXd measures;
  
  if(gnss.header.stamp.sec != 0 && usbl.header.stamp.sec != 0){
    R << gnss_sensor.noise(0,0), 0.0, 0.0, 0.0,
         0.0, gnss_sensor.noise(1,1), 0.0, 0.0,
         0.0, 0.0, usbl_sensor.noise(0,0), 0.0,
         0.0, 0.0, 0.0, usbl_sensor.noise(1,1);

    C << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    measures << gnss.value[0],
                gnss.value[1],
                usbl.value[0],
                usbl.value[1];

    identity << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
  }else if(gnss.header.stamp.sec != 0){
    R = gnss_sensor.noise;

    C << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    measures << gnss.value[0],
                gnss.value[1];

    identity << 1.0, 0.0,
                0.0, 1.0;
  }else if(usbl.header.stamp.sec != 0){
    R = usbl_sensor.noise;

    C << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    measures << usbl.value[0],
                usbl.value[1];

    identity << 1.0, 0.0,
                0.0, 1.0;
  }

  Eigen::MatrixXd K_k, invert_K;
  
  // calculate kalman gains
  invert_K = C * predict_cov_ * C.transpose() + R;
  K_k = predict_cov_ * C.transpose() * invert_K.inverse();

  // calculate state correction and covariance matrix
  state_ = state_ + K_k * (measures - C * state_);
  update_cov_ = (identity - K_k * C) * predict_cov_;
}


// @.@ Where the magic should happen.
void CkfNode::timerIterCallback(const ros::TimerEvent &event) {
  ros::Time new_time = ros::Time::now();
  double delta_t = (new_time - kf_time).toSec();

  kf_time = new_time;
  predict(delta_t);
  update();

  // publish state
  auv_msgs::NavigationStatus msg;
  msg.position.east = state_(0);
  msg.position.north = state_(1);
  msg.body_velocity.x = state_(2);
  msg.body_velocity.y = state_(3);
  msg.status = 59;
  state_pub_.publish(msg);
}


/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "ckf_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type CkfNode");

  // +.+ instantiate an CkfNode class object and pass in pointers to nodehandle public and private for constructor to use
  CkfNode ckf(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
