#include "DeadReckoning.h"

DeadReckoning::DeadReckoning(ros::NodeHandle *nh, ros::NodeHandle *nh_private)
  : nh_(*nh), nh_private_(*nh_private) {

    initializeSubscribers();
    initializePublishers();
    initializeServices();
    initialized_ = false;
    
    // +.+ Read dvl frame from config file
    p_dvl_body_frame_ = FarolGimmicks::getParameters<bool>(
    nh_private_, "dvl/body_frame", true);
  }

// @.@ Destructor
DeadReckoning::~DeadReckoning() {
  // +.+ shutdown publishers
  state_dr_pub_.shutdown();

  // +.+ shutdown subscribers
  sub_velocity_.shutdown();
  sub_orientation_.shutdown();
  sub_true_state_.shutdown();
  flag_sub_.shutdown();

  // +.+ shutdown node
  nh_.shutdown();
}

// @.@ Member Helper function to set up subscribers;
void DeadReckoning::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for DeadReckoning");
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics_dr/velocity", "/measurement/velocity"), 10, &DeadReckoning::velocityCallback, this);

  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics_dr/orientation", "/measurement/orientation"), 10, &DeadReckoning::orientationCallback, this);

  sub_true_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics_dr/true_state", "/nav/filter/state"), 10, &DeadReckoning::stateCallback, this);

  flag_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics_dr/flag", "/Flag"), 10, &DeadReckoning::flagCallback, this);
}

// @.@ Member helper function to set up publishers;
void DeadReckoning::initializePublishers() {
  ROS_INFO( "Initializing Publishers for DeadReckoning"); 

  state_dr_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics_dr/dead_reckoning", "/nav/filter/state_dr"), 10);
  
  state_dr_console_pub_ = nh_private_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics_dr/dead_reckoning_console", "/State_dr"), 10);
}

// @.@ Intialize Services
void DeadReckoning::initializeServices(){

  reset_filter_dr_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, "services_dr/reset_filter_dr", "/nav/reset_filter_dr"), &DeadReckoning::resetDRService, this);

}

// @.@ Get next DR position
void DeadReckoning::computePredict() {
  
  // +.+ Check initialize
  if (!initialized_)
    return;

  double t_request = (ros::Time::now() - last_predict_).toSec();
  // +.+ State pure dead reckoning
  predict(state_dr_, t_request);
  last_predict_ = ros::Time::now();

  // +.+ Return horizontal state estimate
	state_dr_msg_.header.stamp = last_predict_;
  state_dr_msg_.position.north = state_dr_[0];
  state_dr_msg_.position.east = state_dr_[1];
  state_dr_msg_.seafloor_velocity.x = dvl_vx_;
  state_dr_msg_.seafloor_velocity.y = dvl_vy_;
  state_dr_msg_.orientation.x = roll_;
  state_dr_msg_.orientation.y = pitch_;
  state_dr_msg_.orientation.z = yaw_;

	// Set Header Information
	state_dr_console_.header.stamp = last_predict_;
	// Set Position
  state_dr_console_.X = state_dr_[1];
  state_dr_console_.Y = state_dr_[0];
  
	// Set Orientation
  state_dr_console_.Roll = roll_;
	state_dr_console_.Pitch = pitch_;
	state_dr_console_.Yaw = yaw_;

	// Set Velocity
	state_dr_console_.Vx = dvl_vx_;
	state_dr_console_.Vy = dvl_vy_;
	
  // +.+ Publish state dr
  state_dr_pub_.publish(state_dr_msg_);
  // +.+ Publish state dr for console
  state_dr_console_pub_.publish(state_dr_console_);
}

// @.@ Initialize DR
bool DeadReckoning::initialize() {
  // +.+ Set state vector
  state_dr_ = true_state_; 

  last_predict_ = ros::Time::now();
	
	ROS_WARN("Dead Reckoning: initialized with position [%.2f, %.2f]", state_dr_[0], state_dr_[1]);
  return true;
}

// @.@ DR position
void DeadReckoning::predict(std::vector<double> &state_vec, double dt) {
  // +.+ Propagate State
  state_vec[0] = state_dr_[0] + dt * dvl_vx_;
  state_vec[1] = state_dr_[1] + dt * dvl_vy_;
}

// @.@ Callbacks Section / Methods
void DeadReckoning::velocityCallback(const dsor_msgs::Measurement &msg) {
  // +.+ Check if velocity measurement is from dvl
  if (msg.header.frame_id.find("dvl_bt") != std::string::npos){
    // Convert body velocity to inercial velocity
    if(p_dvl_body_frame_){
      dvl_vx_ = cos(DEG2RAD(yaw_))*msg.value[0]  - sin(DEG2RAD(yaw_))*msg.value[1];
      dvl_vy_ = sin(DEG2RAD(yaw_))*msg.value[0] + cos(DEG2RAD(yaw_))*msg.value[1];
    }else{
      dvl_vx_ = msg.value[0];
      dvl_vy_ = msg.value[1];
    }
  }
  //+.+ Compute next DR position
  computePredict();
}

void DeadReckoning::orientationCallback(const dsor_msgs::Measurement &msg) {
  
  // +.+ Check if measurement is from ahrs
  if (msg.header.frame_id.find("ahrs") != std::string::npos){
    roll_ = RAD2DEG(FarolGimmicks::wrap2pi(msg.value[0], 0));
    pitch_ = RAD2DEG(FarolGimmicks::wrap2pi(msg.value[1], 0));
    yaw_ = RAD2DEG(FarolGimmicks::wrap2pi(msg.value[2], 0));
  }
}

// @.@ State from the navigation filter
void DeadReckoning::stateCallback(const auv_msgs::NavigationStatus &msg) {
  true_state_[0] = msg.position.north;
  true_state_[1] = msg.position.east;
  
  // +.+ To DR position starts with the same position of the navigation filter position (truth position)
  if(initialized_ == false){
    initialized_= initialize();
  }
}

// @.@ Flag to reset DR postion when a mission starts
void DeadReckoning::flagCallback(const std_msgs::Int8 &msg) {
 // +.+ Flag = 6 indicates the start of a mission 
  if (msg.data == 6) {
    ROS_WARN("DEAD RECKONING RESETED");
    initialize();
  }
}

// @.@ Service to manual reset DR position
bool DeadReckoning::resetDRService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

  // +.+ The DR position will be equal to the navigation state position (x,y)
  state_dr_ = true_state_;
  res.success = true;
  res.message = "FILTER_DR reseted";

  return true;
}

