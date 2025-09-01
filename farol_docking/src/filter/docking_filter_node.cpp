// Implementation of the DockingFilterNode class
// Author: Ravi Regalo
// Source: Instituto Superior Técnico
// Description: Handles interface between ROS and the docking filter algorithm
#include <farol_docking/filter/docking_filter_node.hpp>  


// Helper function to load a parameter that is a matrix
Eigen::MatrixXd load_matrix_parameter(ros::NodeHandle &_nh, std::string const &parameter_name, Eigen::MatrixXd default_value){
		Eigen::MatrixXd parameter = default_value;
		std::vector<double> temp;
    if (_nh.getParam(parameter_name, temp)) {
        int size = std::sqrt(temp.size()); // Matrix is square, so size = sqrt(vector length)
        if (size * size == temp.size()) {
            Eigen::MatrixXd parameter = Eigen::Map<Eigen::MatrixXd>(temp.data(), size, size);
            // ROS_IFO_STREAM("Loaded process covariance matrix:\n" << parameter);
        } else {
            ROS_ERROR("Invalid process covariance matrix size.");
        }
    } else {
        ROS_ERROR("Failed to load process covariance.");
    }
    return parameter;
}


// Constructor
DockingFilterNode::DockingFilterNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {
  docking_filter_= std::make_unique<DockingFilter>(&nh_,&nh_private_);
  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();
  timer_.start();
  docking_filter_->start(); 
}

// Destructor
DockingFilterNode::~DockingFilterNode() {

  // shutdown publishers
  state_pub_.shutdown();

  // shutdown subscribers
  sub_velocity_.shutdown();
  sub_orientation_.shutdown();
  sub_usbl_fix_.shutdown();
  sub_usbl_accoms_.shutdown();   

  // stop timer
  timer_.stop();

  // shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}


void DockingFilterNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for DockingFilterNode");
  //sub_reset_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "filter/topics/subscribers/reset", "reset"), 10, &DockingFilterNode::resetCallback, this);
  // sub_tuning_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/tuning", "tuning"), 10, &DockingFilterNode::tuningCallback, this);
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &DockingFilterNode::measurement_callback, this);
  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &DockingFilterNode::measurement_callback, this);
  sub_position_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 10, &DockingFilterNode::measurement_callback, this);
  sub_usbl_fix_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/usbl_fix", "usbl_fix"), 10, &DockingFilterNode::usbl_callback, this);
  sub_usbl_accoms_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/usbl_accoms", "usbl_accoms"), 10, &DockingFilterNode::usbl_callback, this);
  sub_reset_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/reset", "reset"), 10, &DockingFilterNode::reset_callback, this);
  sub_terrain_d_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/terrain_normal", "bottom_following/D"), 10, &DockingFilterNode::terrain_normal_callback, this);
}


void DockingFilterNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for DockingFilterNode");
  state_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "docking/filter/state"), 10);
  body_velocity_pub_ = nh_private_.advertise<geometry_msgs::Vector3>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug/body_velocity", "docking/filter/debug/body_velocity"), 10);
}


void DockingFilterNode::initializeServices() {
  ROS_INFO("Initializing Services for DockingFilterNode");
}


void DockingFilterNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / node_frequency_), &DockingFilterNode::timerIterCallback, this);
}


void DockingFilterNode::loadParams() {
  ROS_INFO("Load the DockingFilterNode parameters");
  // ROS related parameters
  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "debug", false);
  
  // Algorithm related parameters
  docking_filter_->initializer_size_ = FarolGimmicks::getParameters<int>(nh_private_, "initializer_size", 4);
  docking_filter_->dock_has_ahrs_ = FarolGimmicks::getParameters<bool>(nh_private_, "dock_has_ahrs", false);

  std::vector<double> aux;
  aux = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "dock_usbl_instalation_offset", {});
  docking_filter_->dock_usbl_instalation_offset << aux[0], aux[1], aux[2];
  aux = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "auv_usbl_instalation_offset", {});
  docking_filter_->auv_usbl_instalation_offset << aux[0], aux[1], aux[2];

  // Filter covariances
  Eigen::MatrixXd noise;
  noise = load_matrix_parameter(nh_private_, "position/process_noise", Eigen::Matrix3d::Identity());
  docking_filter_->configure("position_process", noise);
  noise = load_matrix_parameter(nh_private_, "position/measurement_noise", Eigen::Matrix3d::Identity());
  docking_filter_->configure("position_measurement", noise);
  //noise = load_matrix_parameter(nh_private_, "attitude/process_noise", Eigen::Matrix3d::Identity());
  //docking_filter_->configure("attitude_process", noise);
  //noise = load_matrix_parameter(nh_private_, "attitude/measurement_noise", Eigen::Matrix3d::Identity());
  //docking_filter_->configure("attitude_measurement", noise);


  // outlier rejection config
  std::vector<std::string> outlier_rejection_config;
  outlier_rejection_config = FarolGimmicks::getParameters<std::vector<std::string>>(nh_private_, "outlier_rejection", {});
  docking_filter_->configure("outlier_rejection", outlier_rejection_config);

  // outlier rejection treshold value
  docking_filter_->position_outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "position/outlier_treshold", 4.61);
  docking_filter_->attitude_outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/outlier_treshold", 2.71);

  // load attitude filter parameters
  docking_filter_->attitude_filter_->k1_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/gains/k1", 0.5);
  docking_filter_->attitude_filter_->k2_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/gains/k2", 0.5);
  docking_filter_->attitude_filter_->kp_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/gains/kp", 1);
  docking_filter_->attitude_filter_->ki_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/gains/ki", 0);


  docking_filter_->position_filter_->update_delay_ = FarolGimmicks::getParameters<double>(nh_private_, "position/update_delay", 0.0);
  docking_filter_->attitude_filter_->update_delay_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/update_delay", 0.0);

}


void DockingFilterNode::reset_callback(const std_msgs::Empty &msg){
  docking_filter_->reset();
}


void DockingFilterNode::measurement_callback(const dsor_msgs::Measurement &msg) {
  // Measurements from the AHRS -> extract angular velocities
  if (msg.header.frame_id.find("ahrs") != std::string::npos && msg.value.size() == 6) 
  {
    // drop message if filter not initialized
    if(!docking_filter_->initialized_) 
      return; 
    // send measurement into the docking filter   
    if( docking_filter_->measurements_buffer_.push(Measurement(Eigen::Vector3d(msg.value[3], msg.value[4], msg.value[5]), msg.header.stamp.toSec(), "ahrs_rates")))
        docking_filter_->measurements_buffer_cond_var_.notify_one();
    // no space on buffer, tenso
    else
      ROS_WARN_STREAM("Dropping AHRS measurements. Oh no, not good :(");

    ahrs_velocity_ << msg.value[3],msg.value[4],msg.value[5];
  } 
  // Measurements from the DVL -> extract linear velocities
  else if (msg.header.frame_id.find("dvl") != std::string::npos && msg.value.size() == 3) 
  {
    if(!docking_filter_->initialized_) // keep only last message if not initialized
      return;

    // added - sign because navquest DVL is stoopid
    dvl_velocity_ << msg.value[0],msg.value[1],msg.value[2];
    // rotate 
    dvl_velocity_ = dvl_velocity_ - ahrs_velocity_.cross(r_dvl_);
    body_velocity_pub_.publish(toMsg(dvl_velocity_));

    // send measurement into the docking filter   
    if(docking_filter_->measurements_buffer_.push(Measurement(dvl_velocity_, msg.header.stamp.toSec(), "dvl")))
      docking_filter_->measurements_buffer_cond_var_.notify_one();
    
    // no space on buffer, tenso
    else
      ROS_WARN_STREAM("Dropping DVL measurements. Oh no, not good :(");
  } 
}


void DockingFilterNode::usbl_callback(const farol_msgs::mUSBLFix &msg){
  // if the usbl measurement is made by the vehicle itself
  if(msg.header.frame_id.find("usbl") != std::string::npos){
    // if its a message with range
    if(msg.type == 0){
      usbl_set_.segment<1>(0) << msg.range;
      usbl_state_.set(0, true);
      usbl_times_[0] = ros::Time::now().toSec();
      // ROS_INFO_STREAM("DOCKING::usbl_set_0: "<< std::fixed << std::setprecision(6)<<usbl_times_[0]);
    }
    // if its a message with bearing and elevation
    else if (msg.type == 1){
       if(ignore_first_be_auv_){
        ignore_first_be_auv_=false;
        return;
      }
      usbl_set_.segment<2>(1) << msg.bearing_body, msg.elevation_body;
      usbl_state_.set(1, true);
      usbl_times_[1] = ros::Time::now().toSec();
      // ROS_INFO_STREAM("DOCKING::usbl_set_1: "<< std::fixed << std::setprecision(6)<<usbl_times_[1]);
    }
    
  // if the usbl measurement was made by the dock and then received via accoustic comms
  }else{
    // if its a message with range
    if(msg.type == 0){
      usbl_set_.segment<1>(3) << msg.range;
      usbl_state_.set(2, true);
      usbl_times_[2] = ros::Time::now().toSec();
      // ROS_INFO_STREAM("DOCKING::msg.range: "<< std::fixed << std::setprecision(6)<<usbl_times_[2]);
    }
    // if its a message with bearing and elevation
    else if (msg.type == 1){
      if(ignore_first_be_dock_){
        ignore_first_be_dock_=false;
        return;
      }
      usbl_set_.segment<2>(4) << msg.bearing_body, msg.elevation_body;
      usbl_state_.set(3, true);
      usbl_times_[3] = ros::Time::now().toSec();
      // ROS_INFO_STREAM("DOCKING::msg.be: "<< std::fixed << std::setprecision(6)<<usbl_times_[3]);
    }
  }

  // check if a full usbl set has been received
  if(usbl_state_.all()){
    // check if timestamps of all message match, aka they are all from this interrogration cycle
    if((*std::max_element(usbl_times_.begin(), usbl_times_.end()) - *std::min_element(usbl_times_.begin(), usbl_times_.end())) < 0.35){
      // push measurement into the buffer
      // timestamp is chosen to be the usbl_angles from the auv, which is usually the last message to be received
      if(docking_filter_->measurements_buffer_.push(Measurement(usbl_set_, usbl_times_[1], "usbl")))
        docking_filter_->measurements_buffer_cond_var_.notify_one();
      else // no space on buffer, tenso
        ROS_WARN_STREAM("Dropping USBL measurements. Oh no, not good :(");
    }else{
      ROS_ERROR_STREAM("usbl_messages are from diferent times, diference is "<< (*std::max_element(usbl_times_.begin(), usbl_times_.end()) - *std::min_element(usbl_times_.begin(), usbl_times_.end())) << " seconds." );
      usbl_state_.set(std::distance(usbl_times_.begin(), std::min_element(usbl_times_.begin(), usbl_times_.end())), false);
    }
    //TODO: perhaps change to droping just some messages which are from the past interrogation cycle? 
    
    usbl_state_.reset();
    // ROS_INFO_STREAM("DOCKING::usbl_reset: "<< std::fixed << std::setprecision(6)<<ros::Time::now().toSec());
  }
}


void DockingFilterNode::terrain_normal_callback(const geometry_msgs::Vector3 &msg){
  docking_filter_->terrain_normal_ << msg.x, msg.y, msg.z;
}


void DockingFilterNode::timerIterCallback(const ros::TimerEvent &event) {

  // Proper initialization of the filter using the median of the first 5 measurements
  if(!docking_filter_->initialized_){
    return;
  }

  // Predict the state until current time
  // if(!docking_filter_.predict(event.current_real.toSec())){
  // return;
  // }
  // docking_filter_->predict(event.current_real.toSec());

  state_ = docking_filter_->get_state();

  // publish the estimated state
  state_msg_.header.stamp = ros::Time::now();
  ++state_msg_.header.seq;
  state_msg_.header.frame_id = "mdock0";
  Eigen::Vector3d position = state_.translation();
  Eigen::Quaterniond quaternion = state_.unit_quaternion();
  Eigen::Vector3d rpy = extractRPY(state_.so3());//.matrix().eulerAngles(0, 1, 2);
  Eigen::Vector3d dframe_velocity = state_.so3().matrix().inverse() * dvl_velocity_;
  state_msg_.local_position.x = position[0];
  state_msg_.local_position.y = position[1];
  state_msg_.local_position.z = position[2];
  state_msg_.local_orientation.x = quaternion.x();
  state_msg_.local_orientation.y = quaternion.y();
  state_msg_.local_orientation.z = quaternion.z();
  state_msg_.local_orientation.w = quaternion.w();
  state_msg_.local_attitude.roll = 180/M_PI*rpy[0];
  state_msg_.local_attitude.pitch = 180/M_PI*rpy[1];
  state_msg_.local_attitude.yaw = 180/M_PI*rpy[2];
  state_msg_.body_velocity.x = dvl_velocity_[0];
  state_msg_.body_velocity.y = dvl_velocity_[1];
  state_msg_.body_velocity.z = dvl_velocity_[2];
  state_msg_.seafloor_velocity.x = dframe_velocity[0];
  state_msg_.seafloor_velocity.y = dframe_velocity[1];
  state_msg_.seafloor_velocity.z = dframe_velocity[2];
  state_msg_.orientation_rate.x = 180/M_PI*ahrs_velocity_[0];
  state_msg_.orientation_rate.y = 180/M_PI*ahrs_velocity_[1];
  state_msg_.orientation_rate.z = 180/M_PI*ahrs_velocity_[2];
  state_pub_.publish(state_msg_);
  return;
}


// Main  
int main(int argc, char** argv)
{
  // node set up:
  ros::init(argc, argv, "docking_filter_node"); //node name

  // node handle
  ros::NodeHandle nh;

  // private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type DockingFilterNode");

  // instantiate an DockingFilterNode class object and pass in pointers to nodehandle public and private for constructor to use
  DockingFilterNode df(&nh,&nh_private);

  // Going into spin: let the callbacks do all the work
  ros::spin();

  return 0;
}


