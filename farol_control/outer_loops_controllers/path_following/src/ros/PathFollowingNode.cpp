#include "PathFollowingNode.h"

/**
 * @brief  The Path Following Node constructor
 *
 * @param nh  Public ros node handle
 * @param nh_p  Private ros node handle
 */
PathFollowingNode::PathFollowingNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
    : nh_(*nh), nh_p_(*nh_p) {

  ROS_INFO("in class constructor of PathFollowingNode");
  this->initializeSubscribers();
  this->initializePublishers();
  /* NOTE: initializeServices is implemented inside PathFollowingServices.cpp */
  this->initializeServices();
  this->initializeTimer();

  /* Allocate memory for the default Path Following Algorithm - Brevik */
  this->pf_algorithm_ = getDefaultControllerBrevik();
  pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/pfollowing_debug"),1));
}

/**
 * @brief  Node class destructor
 */
PathFollowingNode::~PathFollowingNode() {

  /* Shutdown all the publishers and deleting the memory allocated for the PF
   * controller */
  this->deleteCurrentController();

  /* Shutdown the subscribers common to all controllers */
  this->state_sub_.shutdown();
  this->path_sub_.shutdown();
  this->vc_sub_.shutdown();

  /* Shutdown the publishers common to all controllers */
  this->flag_pub_.shutdown();
  
  /* Stop the timer callback */
  this->timer_.stop();

  /* Shutdown the node */
  this->nh_.shutdown();
}

/**
 * @brief  Alocates memory for the default controller to be used. In this case
 * is Lapierre
 */
PathFollowing *PathFollowingNode::getDefaultControllerLapierre() {

  /* Get the topic names for the subscribers */
  std::string surge_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string yaw_rate_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/yaw_rate");
  std::string rabbit_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/rabbit");

  /* Create the subscribers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(yaw_rate_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(rabbit_topic, 1));

  /* Read the gains for the controller */
  double k1, k2, k3, theta, k_delta;
  nh_p_.getParam("controller_gains/lapierre/k1", k1);
  nh_p_.getParam("controller_gains/lapierre/k2", k2);
  nh_p_.getParam("controller_gains/lapierre/k3", k3);
  nh_p_.getParam("controller_gains/lapierre/theta", theta);
  nh_p_.getParam("controller_gains/lapierre/k_delta", k_delta);

  /* Return the Path Following object */
  return new Lapierre(k1, k2, k3, theta, k_delta, this->publishers_[0],
                      this->publishers_[1], this->publishers_[2]);
}

/**
 * @brief Alocates memory for a default controller. In this case is Brevik
 */
PathFollowing *PathFollowingNode::getDefaultControllerBrevik() {
  
  /* Get the topic names for the publishers */
  std::string surge_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string yaw_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/yaw");
  std::string rabbit_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/rabbit");
  std::string pfollowing_debug_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pfollowing_debug");

  /* Create the publishers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(yaw_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(rabbit_topic, 1));
 
  /* Assign the new controller */
  return new Brevik(this->publishers_[0], 
        this->publishers_[1], this->publishers_[2]);
}

PathFollowing *PathFollowingNode::getDefaultControllerAguiar() {
  /* Get the topic names for the publishers */
  std::string surge_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string yaw_rate_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/yaw_rate");
  std::string rabbit_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/rabbit");
  std::string pfollowing_debug_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pfollowing_debug");

  /* Create the publishers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(yaw_rate_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(rabbit_topic, 1));

  double delta, kz;
  double kk[2];
  double k_pos;
  double k_currents;

  /* Read the gains for the controller */
  nh_p_.getParam("controller_gains/aguiar/delta", delta);
  nh_p_.getParam("controller_gains/aguiar/kx", kk[0]);
  nh_p_.getParam("controller_gains/aguiar/ky", kk[1]);
  nh_p_.getParam("controller_gains/aguiar/kz", kz);
  nh_p_.getParam("controller_gains/aguiar/k_pos", k_pos);
  nh_p_.getParam("controller_gains/aguiar/k_currents", k_currents);

  /* Assign the new controller */
  return new Aguiar(delta, kk, kz, k_pos, k_currents, this->publishers_[0],
          this->publishers_[1], this->publishers_[2]);
}

/**
 * @brief  Deletes safely the current path following object
 */
void PathFollowingNode::deleteCurrentController() {

  /* Delete the path following object */
  if (this->pf_algorithm_) {
    delete this->pf_algorithm_;
    this->pf_algorithm_ = nullptr;
  }

  /* Clear the publishers vector used by the current path following algorithm */
  for (unsigned int i = 0; i < this->publishers_.size(); i++) {
    this->publishers_[i].shutdown();
  }

  /* Clear the subscribers vector used by the current path following algorithm
   */
  for (unsigned int i = 0; i < this->subscribers_.size(); i++) {
    this->subscribers_[i].shutdown();
  }

  /* Empty the list of publishers and subscribers */
  this->publishers_.clear();
  this->subscribers_.clear();
}

/**
 * @brief  Initialize all the subscribers
 */
void PathFollowingNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for PathFollowingNode");

  /* Get the topic name for the subscribers */
  std::string state_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/subscribers/state");
  std::string path_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/subscribers/path");
  std::string vc_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/subscribers/vc");
  std::string flag_topic =
      FarolGimmicks::getParameters<std::string>(this->nh_p_, "flag");

  /* Initialize the subscribers */
  this->state_sub_ = nh_.subscribe(
      state_topic, 10, &PathFollowingNode::vehicleStateCallback, this);
  this->path_sub_ = nh_.subscribe(path_topic, 10,
                                  &PathFollowingNode::pathStateCallback, this);
  this->vc_sub_ =
      nh_.subscribe(vc_topic, 10, &PathFollowingNode::vcCallback, this);
  this->flag_sub_ =
      nh_.subscribe(flag_topic, 10, &PathFollowingNode::flagCallback, this);
}

/**
 * @brief  Method to initialize the publishers of the path following node
 */
void PathFollowingNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for PathFollowingNode");

  /* Get the topic name for the publishers */
  std::string flag_topic =
      FarolGimmicks::getParameters<std::string>(this->nh_p_, "flag");
  
  /* Initialize the publishers */
  this->flag_pub_ = nh_.advertise<std_msgs::Int8>(flag_topic, 1);
}

/**
 * @brief  Vc callback used to update the velocity correction for cooperative
 * path following
 *
 * @param msg  A pointer to an std_msgs::Float64 with the synchronization speed
 * for the virtual target gamma
 */
void PathFollowingNode::vcCallback(const std_msgs::Float64 &msg) {

  /* Update the desired synchronization correction term */
  this->path_state_.vc = msg.data;
}

/**
 * @brief  Path state callback to update the path data
 *
 * @param msg  A pointer to a dsor_paths::PathData that contains information of
 * the path
 */
void PathFollowingNode::pathStateCallback(const dsor_paths::PathData &msg) {

  /* If the algorithm is running, signal that we have received data from the
   * path */
  if (this->timer_.hasStarted()) {
    has_received_path_state = true;
  }

  /* Update the gamma used to make the computations */
  this->path_state_.gamma = msg.gamma;

  /* Update the path position, derivative and second derivative */
  this->path_state_.pd << msg.pd[0], msg.pd[1], msg.pd[2];
  this->path_state_.d_pd << msg.d_pd[0], msg.d_pd[1], msg.d_pd[2];
  this->path_state_.dd_pd << msg.dd_pd[0], msg.dd_pd[1], msg.dd_pd[2];

  /* Update the angle of the tangent to the path */
  this->path_state_.psi = msg.tangent;
  this->path_state_.curvature = msg.curvature;
  this->path_state_.tangent_norm = msg.derivative_norm;

  /* The desired velocity is the combination of the path speed + coordination
   * speed */
  this->path_state_.vd = msg.vd;
  this->path_state_.d_vd = msg.d_vd;
  this->path_state_.vehicle_speed = msg.vehicle_speed;

  /* Update the bounds of the path following path parameter gamma */
  this->path_state_.gamma_min = msg.gamma_min;
  this->path_state_.gamma_max = msg.gamma_max;
}

/**
 * @brief  Vehicle state callback to update the state of the vehicle
 *
 * @param msg  A pointer to a auv_msgs::NavigationStatus that contains
 * information regarding the vehicle
 */
void PathFollowingNode::vehicleStateCallback(
    const auv_msgs::NavigationStatus &msg) {

  /* If the algorithm is running, signal that we have received data from the
   * vehicle state */
  if (this->timer_.hasStarted()) {
    has_received_vehicle_state = true;
  }

  /* Update the vehicle position */
  this->vehicle_state_.eta1 << msg.position.north, msg.position.east,
      msg.altitude;

  /* Update the vehicle orientation (converting from deg to rad) */
  double roll = FarolGimmicks::wrap2pi(msg.orientation.x * M_PI / 180, 1);
  double pitch = FarolGimmicks::wrap2pi(msg.orientation.y * M_PI / 180, 1);
  double yaw = FarolGimmicks::wrap2pi(msg.orientation.z * M_PI / 180, 1);
  this->vehicle_state_.eta2 << roll, pitch, yaw;

  /* Update the vehicle linear velocity */
  this->vehicle_state_.v1 << msg.body_velocity.x, msg.body_velocity.y,
      msg.body_velocity.z;

  /* Update the vehicle angular velocity (converting from deg to rad) */
  this->vehicle_state_.v2 << msg.orientation_rate.x * M_PI / 180,
      msg.orientation_rate.y * M_PI / 180, msg.orientation_rate.z * M_PI / 180;
}

/**
 * @brief  Initialize the timer callback
 */
void PathFollowingNode::initializeTimer() {
  this->timer_ =
      nh_.createTimer(ros::Duration(1.0 / PathFollowingNode::nodeFrequency()),
                      &PathFollowingNode::timerIterCallback, this);

  /* Wait for the start service to start the Path Following */
  this->timer_.stop();
}

/**
 * @brief  Setup the Node working frequency
 */
double PathFollowingNode::nodeFrequency() {

  double node_frequency =
      FarolGimmicks::getParameters<double>(this->nh_p_, "node_frequency", 10);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

/**
 * @brief Method where the logic is located in order to update the control law
 *
 * @param event  A TimerEvent from ros
 */
void PathFollowingNode::timerIterCallback(const ros::TimerEvent &event) {

  /* If it has not received data from the path or the vehicle, do not update the
   * algorithm */
  if (!this->has_received_path_state || !this->has_received_vehicle_state)
    return;

  /* Update the values inside the PathFollowing controller */
  this->pf_algorithm_->UpdateVehicleState(this->vehicle_state_);
  this->pf_algorithm_->UpdatePathState(this->path_state_);

  /* Get the difference between previous update time and current update time */
  ros::Time curr_time = ros::Time::now();
  ros::Duration dt = curr_time - this->prev_time_;
  this->prev_time_ = curr_time;

  /* Compute the control law */
  this->pf_algorithm_->callPFController(double(dt.toSec()));

  /* Publish the results */
  this->pf_algorithm_->publish();

  /* Check if we have reached the end of the path */
  if (this->pf_algorithm_->stop()) {
    /* Ask waypoint algorithm to hold position */
    this->sendWaypoint(WP_FINISH);
    /* Reset the DR postion to the 2d state filter position */
    this->sendResetDeadReckoning();

  }
}

/**
 * @brief  Auxiliar method to send a hold position waypoint
 *
 * @param value -3 to hold the pos after the vehicle, -1 to hold position on the spot
 */
void PathFollowingNode::sendWaypoint(double value) {
  waypoint::sendWpType1 srv;
  srv.request.x = value;
  srv.request.y = value;
  wp_standard_client_.call(srv);
}

/**
 * @brief  Auxiliar method to reset the DeadReckoning postion  
 * */
void PathFollowingNode::sendResetDeadReckoning() {
  std_srvs::Trigger srv;
  dr_reset_client_.call(srv);
}

/**
 * @brief  Auxiliar method to stop the algorithm
 */
void PathFollowingNode::stopAlgorithm() {

  /* Inform the user that path following will stop */
  ROS_INFO("Path Following Node has stop.");

  /* Stop the timer callback where the pf algorithm is invoked */
  this->timer_.stop();

  /* Reset the Path following controller */
  this->pf_algorithm_->reset();

  /* Reset the flags used in the first iteration */
  this->has_received_vehicle_state = false;
  this->has_received_path_state = false;

  /* Call the service to reset the path */
  dsor_paths::ResetPath srv;
  srv.request.reset_path = true;
  reset_path_client_.call(srv);
}

/**
 * @brief  Auxiliar method to stop the algorithm
 */
void PathFollowingNode::flagCallback(const std_msgs::Int8 &msg) {

  /* If the flag changed and the path following was running, then stop */
  if (msg.data != FLAG_PF && this->timer_.hasStarted()) {
    this->stopAlgorithm();
  }
}

/**
 * @brief  The main function
 *
 * @param argc  The argument count
 * @param argv  The argument array
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "path_following_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  ROS_INFO("main: instantiating an object of type PathFollowingNode");

  /* Instantiate the PathFollowing Node*/
  PathFollowingNode PathFollowing(&nh, &nh_p);

  /* Going into spin and let the timer callback do all the work */
  ros::spin();

  return 0;
}
