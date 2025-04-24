/* 
 * Developer: Ravi Regalo -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "DockingControllerNode.h"

// @.@ Constructor
DockingControllerNode::DockingControllerNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

}

// @.@ Destructor
DockingControllerNode::~DockingControllerNode() {

  // +.+ shutdown publishers
  surge_ref_pub_.shutdown();
  sway_ref_pub_.shutdown();
  yaw_ref_pub_.shutdown();
  flag_pub_.shutdown();

  // +.+ shutdown subscribers
  sub_velocity_.shutdown();
  sub_orientation_.shutdown();
  sub_state_.shutdown();
  sub_start_.shutdown();
  sub_flag_.shutdown();


  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void DockingControllerNode::loadParams() {
  ROS_INFO("Load the DockingControllerNode parameters");
  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);
  controller_.fully_actuated_ = FarolGimmicks::getParameters<bool>(nh_private_, "fully_actuated", false);
  debug_ = FarolGimmicks::getParameters<bool>(nh_private_, "debug", false);
  // params_ = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "params", false);
  nh_private_.getParam("params", params_);
  controller_.tune(params_);
}


// @.@ Member helper function to set up subscribers
void DockingControllerNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for DockingControllerNode");
  sub_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state", "docking_state"), 10, &DockingControllerNode::state_callback, this);
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &DockingControllerNode::measurement_callback, this);
  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &DockingControllerNode::measurement_callback, this);
  sub_start_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/enable", "enable"), 10, &DockingControllerNode::start_callback, this);
  sub_flag_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/flag", "flag"), 10, &DockingControllerNode::flag_callback, this);
  sub_dock_inertial_pos_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/dock_pose", "dock_pose"), 10, &DockingControllerNode::dock_pose_callback, this);
  sub_filter_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/filter_state", "/nav/filter/state"), 10, &DockingControllerNode::filter_state_callback, this);
}


// @.@ Member helper function to set up publishers
void DockingControllerNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for DockingControllerNode");
  surge_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_surge", "ref/surge"), 5);
  sway_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_sway", "ref/sway"), 5);
  yaw_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_yaw", "ref/yaw"), 5);
  depth_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ref_depth", "ref/depth"), 5);
  flag_pub_ = nh_private_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/flag", "flag"), 5);
  debug_pub_ = nh_private_.advertise<farol_docking::ControllerDebug>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug", "debug"), 5);

}


// @.@ Member helper function to set up services
void DockingControllerNode::initializeServices() {
  ROS_INFO("Initializing Services for DockingControllerNode");

}


// @.@ Member helper function to set up the timer
void DockingControllerNode::initializeTimer() {
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &DockingControllerNode::timerIterCallback, this);
}

void DockingControllerNode::state_callback(const farol_docking::dState &msg){
  controller_.x_ = msg.x;
  controller_.y_ = msg.y;
  controller_.z_ = msg.z;
  controller_.yaw_ = msg.yaw;
}

void DockingControllerNode::measurement_callback(const dsor_msgs::Measurement &msg) {
  if (msg.header.frame_id.find("ahrs") != std::string::npos && !msg.value.empty()) {
      ahrs_ = msg.value[0];
  } else if (msg.header.frame_id.find("dvl") != std::string::npos && msg.value.size() >= 2) {
      dvl_ << msg.value[0], msg.value[1];  
      controller_.u_ = msg.value[0];
      controller_.v_ = msg.value[1];
  } 
}

void DockingControllerNode::start_callback(const std_msgs::Empty &msg){
  flag_ = 10;
  flag_msg_.data = 10;
  flag_pub_.publish(flag_msg_);
  controller_.sigma_ = 0;
}

void DockingControllerNode::flag_callback(const std_msgs::Int8 &msg){
  flag_ = msg.data;
}

void DockingControllerNode::dock_pose_callback(const farol_msgs::mState &msg){
  dock_pose_ << msg.Y, msg.X, -msg.Z, msg.Yaw/180*M_PI;
}

// to adjust references
void DockingControllerNode::filter_state_callback(const auv_msgs::NavigationStatus &msg){
  filter_state_ << msg.position.depth, msg.orientation.z;
}

// @.@ Where the magic should happen.
void DockingControllerNode::timerIterCallback(const ros::TimerEvent &event) {
  // compute time interval 
  ros::Time new_time = ros::Time::now();
  double Dt = (new_time - last_update_time_ ).toSec();
  last_update_time_ = new_time;

  // check if mission is over and change the flag
  if(controller_.x_ > 0.1 & controller_.x_ < 2  & !reached_close_){
    end_time_ = new_time;
    reached_close_=true;
    ROS_ERROR_STREAM(end_time_);
  }
  if( (flag_ == 10) && (controller_.x_ < 1.8 || (end_time_ - new_time).toSec() > 10)){
    flag_ = 0;
    flag_msg_.data = 0;
    flag_pub_.publish(flag_msg_);
  }

  // publish debug message
  if(debug_){ 
    debug_msg_.u_ref = controller_.output_(0);
    debug_msg_.v_ref = controller_.output_(1);
    debug_msg_.yaw_ref = controller_.output_(2);
    debug_msg_.heading_ref = controller_.output_(2) + controller_.yaw_ + ahrs_;
    debug_msg_.K1 = controller_.K1_;
    debug_msg_.K2 = controller_.K2_;
    debug_msg_.Ka = controller_.Ka_;
    debug_msg_.sigma = controller_.sigma_;
    debug_msg_.state = controller_.controller_state_;
    debug_msg_.cross_track = controller_.cross_track_;
    debug_msg_.along_track = controller_.x_;
    debug_msg_.sigma_dot = controller_.sigma_dot_;
    debug_msg_.yaw_correction = controller_.yaw_correction_;
    debug_msg_.yaw_d = controller_.yaw_d_;
    debug_msg_.u = controller_.u_;
    debug_msg_.U = controller_.U_;
    debug_pub_.publish(debug_msg_);
  }
  
  // do nothing if the flag is set to 0 (idle mode)
  if(flag_ != 10)
    return;

  // Compute references for inner loops
  if(!controller_.compute_update(Dt))
    return;
  
  // publish reference velocities
  ref_msg_.data = controller_.output_(0);
  surge_ref_pub_.publish(ref_msg_);
  ref_msg_.data = controller_.output_(1);
  sway_ref_pub_.publish(ref_msg_);

  // convert reference yaw to the inertial yaw for inertial pid controller
  ref_msg_.data = (controller_.output_(2) + dock_pose_(3))*180/M_PI;
  //ref_msg_.data = wrapToPi(controller_.output_(2) - controller_.yaw_ +  filter_state_(1)/180*M_PI)*180/M_PI;
  yaw_ref_pub_.publish(ref_msg_);

  // convert reference z to the inertial depth for farol pid controller: 
  // ref_msg_.data = (0 -controller_.z_ + filter_state_(0));
  ref_msg_.data = (0 + dock_pose_(2));
  depth_ref_pub_.publish(ref_msg_);
}


/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "farol_docking_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type DockingControllerNode");

  // +.+ instantiate an DockingControllerNode class object and pass in pointers to nodehandle public and private for constructor to use
  DockingControllerNode farol_docking(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
