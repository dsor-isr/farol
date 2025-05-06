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
  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();
  timer_.start();
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
}


void DockingFilterNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for DockingFilterNode");
  state_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "docking/filter/state"), 10);
  // console_state_pub_ = nh_private_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state_console", "State_docking"), 10);
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
  debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "node_frequency", false);
  
  // Algorithm related parameters
  docking_filter_.initializer_size_ = FarolGimmicks::getParameters<int>(nh_private_, "initializer_size", 4);
  docking_filter_.dock_has_ahrs_ = FarolGimmicks::getParameters<bool>(nh_private_, "dock_has_ahrs", false);

  // Filter covariances
  Eigen::MatrixXd noise;
  noise = load_matrix_parameter(nh_private_, "position/process_noise", Eigen::Matrix3d::Identity());
  docking_filter_.configure("position_process", noise);
  noise = load_matrix_parameter(nh_private_, "position/measurement_noise", Eigen::Matrix3d::Identity());
  docking_filter_.configure("position_measurement", noise);
  noise = load_matrix_parameter(nh_private_, "attitude/process_noise", Eigen::Matrix3d::Identity());
  docking_filter_.configure("attitude_process", noise);
  noise = load_matrix_parameter(nh_private_, "attitude/measurement_noise", Eigen::Matrix3d::Identity());
  docking_filter_.configure("attitude_measurement", noise);


  // outlier rejection config
  std::vector<std::string> outlier_rejection_config;
  outlier_rejection_config = FarolGimmicks::getParameters<std::vector<std::string>>(nh_private_, "outlier_rejection", {});
  docking_filter_.configure("outlier_rejection", outlier_rejection_config);

  // outlier rejection treshold value
  docking_filter_.position_outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "position/outlier_treshold", 4.61);
  docking_filter_.attitude_outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "attitude/outlier_treshold", 2.71);

}


void DockingFilterNode::reset_callback(const std_msgs::Empty &msg){
  docking_filter_.reset();
}

void DockingFilterNode::measurement_callback(const dsor_msgs::Measurement &msg) {
  // Measurements from the AHRS -> extract angular velocities
  if (msg.header.frame_id.find("ahrs") != std::string::npos && msg.value.size() == 6) 
  {
    // drop message if filter not initialized
    if(!docking_filter_.initialized_) 
      return; 
    // send measurement into the docking filter   
    if( docking_filter_.measurements_buffer_.push(Measurement(Eigen::Vector3d(msg.value[3], msg.value[4], msg.value[5]), msg.header.stamp.toSec(), "ahrs_rates")) &&
        docking_filter_.measurements_buffer_.push(Measurement(Eigen::Vector3d(msg.value[0], msg.value[1], msg.value[2]), msg.header.stamp.toSec(), "ahrs_angles")))
    {
      docking_filter_.measurements_buffer_cond_var_.notify_one();
    }
    // no space on buffer, tenso
    else{
      ROS_WARN_STREAM("Dropping AHRS measurements. Oh no, not good :(");
    }
  } 
  // Measurements from the DVL -> extract linear velocities
  else if (msg.header.frame_id.find("dvl") != std::string::npos && msg.value.size() == 3) 
  {
    if(!docking_filter_.initialized_) // keep only last message if not initialized
      return;
      
    // send measurement into the docking filter   
    if(docking_filter_.measurements_buffer_.push(Measurement(Eigen::Vector3d(msg.value[0], msg.value[1], msg.value[2]), msg.header.stamp.toSec(), "dvl"))){
      docking_filter_.measurements_buffer_cond_var_.notify_one();
    }
    // no space on buffer, tenso
    else{
      ROS_WARN_STREAM("Dropping DVL measurements. Oh no, not good :(");
    }
  } 
}

void DockingFilterNode::usbl_callback(const farol_msgs::mUSBLFix &msg){
  // if the usbl measurement is made by the vehicle itself
  if(msg.header.frame_id.find("usbl") != std::string::npos){
    // if its a message with range
    if(msg.type == 0){
      usbl_set_.segment<1>(0) << msg.range;
      usbl_state_.set(0, true);
    }
    // if its a message with bearing and elevation
    else if (msg.type == 1){
      usbl_set_.segment<2>(1) << msg.bearing_body, msg.elevation_body;
      usbl_state_.set(1, true);
    }
    usbl_time_ = msg.header.stamp;
    
  // if the usbl measurement was made by the dock and then received via accoustic comms
  }else{
    // if its a message with range
    dock_frame_id_ = msg.header.frame_id;
    if(msg.type == 0){
      usbl_set_.segment<1>(3) << msg.range;
      usbl_state_.set(2, true);
    }
    // if its a message with bearing and elevation
    else if (msg.type == 1){
      usbl_set_.segment<2>(4) << msg.bearing_body, msg.elevation_body;
      usbl_state_.set(3, true);
      
      // if the dock has an ahrs we need to go get that little bitch
      if(docking_filter_.dock_has_ahrs_)
        if(docking_filter_.measurements_buffer_.push(Measurement(Eigen::Vector3d(msg.ahrs_roll, msg.ahrs_pitch, msg.ahrs_yaw), msg.header.stamp.toSec(), "dock_attitude")))
          docking_filter_.measurements_buffer_cond_var_.notify_one();
   
    }
    usbl_time_ = msg.header.stamp;
  }

  // check if a full usbl set has been received
  if(usbl_state_.all()){
    // add new measurement to the buffer
    if(docking_filter_.measurements_buffer_.push(Measurement(usbl_set_, usbl_time_.toSec(), "usbl"))){
      docking_filter_.measurements_buffer_cond_var_.notify_one();
    }
    // no space on buffer, tenso
    else{
      ROS_WARN_STREAM("Dropping USBL measurements. Oh no, not good :(");
    }
    usbl_state_.reset();
  }
}

void DockingFilterNode::timerIterCallback(const ros::TimerEvent &event) {

  // Proper initialization of the filter using the median of the first 5 measurements
  if(!docking_filter_.initialized_){
    return;
  }

  // Predict the state until current time
  if(!docking_filter_.predict(event.current_real.toSec())){
  return;
  }

  Sophus::SE3d state = docking_filter_.get_state();

  // publish the estimated state
  state_msg_.header.stamp = ros::Time::now();
  ++state_msg_.header.seq;
  state_msg_.header.frame_id = dock_frame_id_;
  Eigen::Vector3d position = state.translation();
  Eigen::Quaterniond quaternion = state.unit_quaternion();
  Eigen::Vector3d rpy = state.so3().matrix().eulerAngles(0, 1, 2);
  state_msg_.local_position.x = position[0];
  state_msg_.local_position.y = position[1];
  state_msg_.local_position.z = position[2];
  state_msg_.local_orientation.x = quaternion.x();
  state_msg_.local_orientation.y = quaternion.y();
  state_msg_.local_orientation.z = quaternion.z();
  state_msg_.local_orientation.w = quaternion.w();
  state_msg_.local_attitude.roll = rpy[0];
  state_msg_.local_attitude.pitch = rpy[1];
  state_msg_.local_attitude.yaw = rpy[2];
  state_pub_.publish(state_msg_);

  //TODO: add the covariances here as well


  // publish state in the inertial NED reference frame 
  // console_state_msg_.header.stamp = ros::Time::now();
  // inertial_pos_ = Rot2D(-dock_pose_(3)) * statevec_.head(2) + dock_pose_.head(2);
  // console_state_msg_.X = inertial_pos_(1);    // East
  // console_state_msg_.Y = inertial_pos_(0);    // North
  // console_state_msg_.Yaw = radiansToDegrees360(wrapToPi(-dock_pose_(3) + statevec_));
  // console_state_msg_.Z = statevec_ + dock_pose_(2);
  // console_state_pub_.publish(console_state_msg_);

  // publish debug msg
  // if(debug_){
  //   debug_msg_.header.stamp = ros::Time::now();

  //   // horizontal filter debug info
  //   inertial_pos_ = Rot2D(-dock_pose_(3)) * observations_.head<2>() + dock_pose_.head(2);
  //   debug_msg_.h_meas_x = observations_(0);
  //   debug_msg_.h_meas_y = observations_(1);
  //   debug_msg_.h_meas_x_NED = inertial_pos_(0);
  //   debug_msg_.h_meas_y_NED = inertial_pos_(1);
  //   debug_msg_.h_mahalanobis = docking_filter_.mahalanobis_distance_;
  //   debug_msg_.h_outlier_rejected = docking_filter_.outlier_rejected_;
  //   debug_msg_.h_innovation_vec[0] = docking_filter_.innovation_vector_(0);
  //   debug_msg_.h_innovation_vec[1] = docking_filter_.innovation_vector_(1);
  //   debug_msg_.h_innovation_mat[0] = docking_filter_.innovation_matrix_(0);
  //   debug_msg_.h_innovation_mat[1] = docking_filter_.innovation_matrix_(1);
  //   debug_msg_.h_innovation_mat[2] = docking_filter_.innovation_matrix_(2);
  //   debug_msg_.h_innovation_mat[3] = docking_filter_.innovation_matrix_(3);
  //   debug_msg_.h_k[0] = docking_filter_.K_(0);
  //   debug_msg_.h_k[1] = docking_filter_.K_(1);
  //   debug_msg_.h_k[2] = docking_filter_.K_(2);
  //   debug_msg_.h_k[3] = docking_filter_.K_(3);
  //   debug_msg_.h_k[4] = docking_filter_.K_(4);
  //   debug_msg_.h_k[5] = docking_filter_.K_(5);
  //   debug_msg_.h_k[6] = docking_filter_.K_(6);
  //   debug_msg_.h_k[7] = docking_filter_.K_(7);

  //   // vertical filter debug info
  //   debug_msg_.v_meas_z = observations_(2);
  //   debug_msg_.v_meas_z_NED = prev_depth_;
  //   debug_msg_.v_mahalanobis = docking_filter_.mahalanobis_distance_;
  //   debug_msg_.v_outlier_rejected = docking_filter_.outlier_rejected_;
  //   debug_msg_.v_innovation_vec = docking_filter_.innovation_vector_;
  //   debug_msg_.v_innovation_mat = docking_filter_.innovation_matrix_;
  //   debug_msg_.v_k = docking_filter_.K_;

  //   // rotational filter debug info
  //   debug_msg_.r_meas_yaw = observations_(3);
  //   debug_msg_.r_meas_yaw_NED = radiansToDegrees360(wrapToPi(-dock_pose_(3) + observations_(3)));
  //   debug_msg_.r_mahalanobis = docking_filter_.mahalanobis_distance_;
  //   debug_msg_.r_outlier_rejected = docking_filter_.outlier_rejected_;
  //   debug_msg_.r_innovation_vec = docking_filter_.innovation_vector_;
  //   debug_msg_.r_innovation_mat = docking_filter_.innovation_matrix_;
  //   debug_msg_.r_k = docking_filter_.K_;
    
  //   debug_pub_.publish(debug_msg_);
  // }
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


