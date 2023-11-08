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

  //initialized_ = true;
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
  std::string sensor_str = FarolGimmicks::getParameters<std::string>(nh_private_, "sensors/list", "gps,usbl,dvl_bt,ahrs");

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
    if(sc.name == "gps"){
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

  init_value_ << 4290822.1983094830,     // Position -         x
                 491906.60293571133,     //                    y
                 0.0000000000000000,     // Current velocity - vcx
                 0.0000000000000000;     //                    vcy

  state_ << init_value_;

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


  complementary_cov = process_cov + 
}


// @.@ Member helper function to set up subscribers
void CkfNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for CkfNode");
  sub_reset_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/reset", "reset"), 10, &CkfNode::resetCallback, this);

  sub_position_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 10, &CkfNode::measurementCallback, this);
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &CkfNode::measurementCallback, this);
  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &CkfNode::measurementCallback, this);
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

}

void CkfNode::resetCallback(const std_msgs::Empty &msg){

}

void CkfNode::predict(){

}

void CkfNode::update(){

}


// @.@ Where the magic should happen.
void CkfNode::timerIterCallback(const ros::TimerEvent &event) {
  
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
