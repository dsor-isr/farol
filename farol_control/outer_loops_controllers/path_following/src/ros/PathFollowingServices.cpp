#include "PathFollowingNode.h"

/**
 * @brief  A method for initializing all the services. This method is called by
 * the constructor of the PathNode class upon creation
 */
void PathFollowingNode::initializeServices() {
  ROS_INFO("Initializing Services for PathFollowingNode");

  /* Get the service names for starting and stoping the path following */
  std::string start_pf_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/start_pf", "/start_pf");
  std::string stop_pf_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/stop_pf", "/stop_pf");
  std::string update_gains_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/updates_gains_pf", "/update_gains_pf");

  /* Get the service names for switching the path following algorithm to use */
  std::string set_marcelo_name = FarolGimmicks::getParameters<std::string>(
    this->nh_p_, "topics/services/marcelo_pf", "/set_marcelo_pf");
  std::string set_aguiar_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/aguiar_pf", "/set_aguiar_pf");
  std::string set_brevik_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/brevik_pf", "/set_brevik_pf");
  std::string set_fossen_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/fossen_pf", "/set_fossen_pf");
  std::string set_romulo_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/romulo_pf", "/set_romulo_pf");
  std::string set_lapierre_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/lapierre_pf", "/set_lapierre_pf");
  std::string set_pramod_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/pramod_pf", "/set_pramod_pf");
  std::string set_samson_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/samson_pf", "/set_samson_pf");
  std::string set_relative_heading_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/relative_heading_pf", "/set_relative_heading_pf");

  /* Get the service name for reseting the virtual target in the path following */
  std::string reset_virtual_taget_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/reset_vt_pf", "/resetVT_pf");
    
  /* Get the service name for waypoint*/
  std::string wp_service_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/wp_standard", "/controls/send_wp_standard");

  /* Get the service name for reset_dr*/
  std::string reset_dr_service_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/reset_dr", "/nav/reset_filter_dr");

  /* Get the name to reset and set the mode of operation of the path */
  std::string reset_path_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/reset_path");
  std::string set_path_mode_name = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/set_path_mode");
  
  /* Advertise the services with these names */
  this->pf_start_srv_ = this->nh_.advertiseService(
      start_pf_name, &PathFollowingNode::StartPFService, this);
  this->pf_stop_srv_ = this->nh_.advertiseService(
      stop_pf_name, &PathFollowingNode::StopPFService, this);
  this->pf_update_gains_srv_ = this->nh_.advertiseService(
      update_gains_name, &PathFollowingNode::UpdateGainsPFService, this);

  this->pf_relative_heading_srv_ = this->nh_.advertiseService(
    set_relative_heading_name, &PathFollowingNode::SetRelativeHeadingService, this);
  this->pf_marcelo_srv_ = this->nh_.advertiseService(
    set_marcelo_name, &PathFollowingNode::SetMarceloService, this);
  this->pf_aguiar_srv_ = this->nh_.advertiseService(
      set_aguiar_name, &PathFollowingNode::SetAguiarService, this);
  this->pf_brevik_srv_ = this->nh_.advertiseService(
      set_brevik_name, &PathFollowingNode::SetBrevikService, this);
  this->pf_fossen_srv_ = this->nh_.advertiseService(
      set_fossen_name, &PathFollowingNode::SetFossenService, this);
  this->pf_romulo_srv_ = this->nh_.advertiseService(
      set_romulo_name, &PathFollowingNode::SetRomuloService, this);
  this->pf_lapierre_srv_ = this->nh_.advertiseService(
      set_lapierre_name, &PathFollowingNode::SetLapierreService, this);
  this->pf_pramod_srv_ = this->nh_.advertiseService(
      set_pramod_name, &PathFollowingNode::SetPramodService, this);
  this->pf_samson_srv_ = this->nh_.advertiseService(
      set_samson_name, &PathFollowingNode::SetSamsonService, this);

  this->pf_reset_vt_srv_ = this->nh_.advertiseService(
      reset_virtual_taget_name, &PathFollowingNode::ResetVirtualTargetService, this);
  
  /* Setup the waypoint client needed when mission finishes */
  this->wp_standard_client_ = nh_.serviceClient<waypoint::sendWpType1>(wp_service_name);
  
  /* Setup the reset DeadReckoning client needed when mission finishes */
  this->dr_reset_client_ = nh_.serviceClient<std_srvs::Trigger>(reset_dr_service_name);
  
  /* Reset the path we are following and set the mode of operation */
  this->reset_path_client_ = nh_.serviceClient<dsor_paths::ResetPath>(reset_path_name);
  this->set_path_mode_client_ = nh_.serviceClient<dsor_paths::SetMode>(set_path_mode_name);
}

/* Service to start running the path following algorithm that was chosen
 * previously */
bool PathFollowingNode::StartPFService(path_following::StartPF::Request &req,
    path_following::StartPF::Response &res) {

  /* Check if we have a path following algorithm allocated. If so, start the
   * timer callbacks */
  if (this->pf_algorithm_ != nullptr) {

    /* Update the last time the iteration of the path following run */
    this->prev_time_ = ros::Time::now();
    this->timer_.start();
    res.success = true;

    /* Publish the code that simbolizes that path following has started */
    FarolGimmicks::publishValue<std_msgs::Int8, const int>(this->flag_pub_,
        FLAG_PF);

    /* Run the first iteration of the algorithm */
    this->pf_algorithm_->start();

    /* Inform the user the path following algorithm will start */
    ROS_INFO("Path Following is starting.");

    return true;
  }

  /* If there is not object for path following allocated, then print message to
   * console */
  ROS_WARN("There is not path following method allocated. Please restart the "
      "node or set the PF to use.");
  res.success = false;
  return true;
}

/* Service to stop the path following algorithm that was running */
bool PathFollowingNode::StopPFService(path_following::StopPF::Request &req,
    path_following::StopPF::Response &res) {

  /* Stop the path following only if it was already running */
  if (this->timer_.hasStarted()) {
    /* Publish the code that simbolizes idle mode */
    FarolGimmicks::publishValue<std_msgs::Int8, const int>(this->flag_pub_,
        FLAG_IDLE);
  }

  /* Return success */
  res.success = true;

  return true;
}

/* Service to reset the virtual target value */
bool PathFollowingNode::ResetVirtualTargetService(path_following::ResetVT::Request &req,
        path_following::ResetVT::Response &res) {

    /* Check if we have a path following algorithm instantiated */
    if(this->pf_algorithm_ == nullptr) {
      res.success = false;
      return true;
    }

    /* Reset the virtual target */
    res.success = this->pf_algorithm_->resetVirtualTarget((float) req.value);
    return true;
}

/* Servce to update the gains of the path following algorithms live */
bool PathFollowingNode::UpdateGainsPFService(
    path_following::UpdateGainsPF::Request &req,
    path_following::UpdateGainsPF::Response &res) {

  /* Get the new gains */
  std::vector<double> new_gains = req.gains;

  /* Pass the new gains for the controller */
  if (this->pf_algorithm_ != nullptr) {
    /* Try to update the gains */
    bool result = this->pf_algorithm_->setPFGains(new_gains);

    /* Inform the user if the new gains were accepted or not */
    if (result == true) {
      ROS_INFO("Gains updated successfully!");
    } else {
      ROS_INFO("Gains not accepted!");
    }

    /* Update the response message */
    res.success = result;
    return true;
  }

  /* If the path following algorithm object is not allocated, some error ocurred
   * and we need to restart this node */
  ROS_WARN("There is not path following method allocated. Please restart the "
      "node or set the PF to use.");
  res.success = false;

  return true;
}

/* Service to switch to the RelativeHeading Path Following method */
bool PathFollowingNode::SetRelativeHeadingService(path_following::SetPF::Request &req, path_following::SetPF::Response &res) {

    /* Don't change if the algorithm is running */
    if (this->timer_.hasStarted()) {
        ROS_INFO("Can't change algorithm when PF is running.");
        res.success = false;
        return true;
    }

    /* Clear the memory used by the previous controller */
    this->deleteCurrentController();

    /* Get the topic names for the publishers */
    std::string surge_topic = FarolGimmicks::getParameters<std::string>(
            this->nh_p_, "topics/publishers/surge");
    std::string sway_topic = FarolGimmicks::getParameters<std::string>(
            this->nh_p_, "topics/publishers/sway");
    std::string yaw_topic = FarolGimmicks::getParameters<std::string>(
            this->nh_p_, "topics/publishers/yaw");
    std::string rabbit_topic = FarolGimmicks::getParameters<std::string>(
            this->nh_p_, "topics/publishers/rabbit");
    std::string pfollowing_debug_topic = FarolGimmicks::getParameters<std::string>(
            this->nh_p_, "topics/publishers/pfollowing_debug");

    /* Create the publishers for the node */
    this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
    this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(sway_topic, 1));
    this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(yaw_topic, 1));
    this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(rabbit_topic, 1));

    /* Read the control gains from the parameter server */
    double kx, ky, kz, yaw_offset;
    std::vector<double> p_sat;

    try {

        /* Read the gains for the controller */
        nh_p_.getParam("controller_gains/relative_heading/kx", kx);
        nh_p_.getParam("controller_gains/relative_heading/ky", ky);
        nh_p_.getParam("controller_gains/relative_heading/kz", kz);
        nh_p_.getParam("controller_gains/relative_heading/yaw_offset", yaw_offset);
        nh_p_.getParam("controller_gains/relative_heading/p_sat", p_sat);

        /* Assign the new controller */
        this->pf_algorithm_ = new RelativeHeading(kx, ky, kz, Eigen::Vector2d(p_sat.data()), yaw_offset, this->publishers_[0], this->publishers_[1], this->publishers_[2], this->publishers_[3]);

        /* Path the debug variables publisher to the class*/
        pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));

        /* Return success */
        res.success = true;

    } catch (...) {
        ROS_WARN("Some error occured. Please reset the PF node for safety");
        res.success = false;
        return false;
    }

    /* Return success */
    ROS_INFO("PF controller switched to RelativeHeading");
    return true;
}

/* Service to switch to the Marcelo Path Following method */
bool PathFollowingNode::SetMarceloService(path_following::SetPF::Request &req,
    path_following::SetPF::Response &res) {

  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PF is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Get the topic names for the publishers */
  std::string surge_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string yaw_rate_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/yaw_rate");
  std::string rabbit_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/rabbit");
  std::string pfollowing_debug_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pfollowing_debug");
  std::string observer_x_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/observer/x");
  std::string observer_y_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/observer/y");

  /* Create the publishers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(yaw_rate_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(rabbit_topic, 1));

  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(observer_x_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(observer_y_topic, 1));

  double delta, kz;
  double kk[2];
  double k_pos;
  double k_currents;
  std::vector<double> rd;
  std::vector<double> d;

  try {

    /* Read the gains for the controller */
    nh_p_.getParam("controller_gains/marcelo/delta", delta);
    nh_p_.getParam("controller_gains/marcelo/kx", kk[0]);
    nh_p_.getParam("controller_gains/marcelo/ky", kk[1]);
    nh_p_.getParam("controller_gains/marcelo/kz", kz);
    nh_p_.getParam("controller_gains/marcelo/k_pos", k_pos);
    nh_p_.getParam("controller_gains/marcelo/k_currents", k_currents);
    nh_p_.getParam("controller_gains/marcelo/rd", rd);
    nh_p_.getParam("controller_gains/marcelo/d", d);

    /* Assign the new controller */
    this->pf_algorithm_ =
      new Marcelo(delta, kk, kz, k_pos, k_currents, rd.data(), d.data(), this->publishers_[0],
          this->publishers_[1], this->publishers_[2], this->publishers_[3], this->publishers_[4]);

    pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));
    
    res.success = true;

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PF node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PF controller switched to Marcelo");
  return true;
}

/* Service to switch to the Aguiar Path Following method */
bool PathFollowingNode::SetAguiarService(path_following::SetPF::Request &req,
    path_following::SetPF::Response &res) {

  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PF is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

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

  try {

    /* Read the gains for the controller */
    nh_p_.getParam("controller_gains/aguiar/delta", delta);
    nh_p_.getParam("controller_gains/aguiar/kx", kk[0]);
    nh_p_.getParam("controller_gains/aguiar/ky", kk[1]);
    nh_p_.getParam("controller_gains/aguiar/kz", kz);
    nh_p_.getParam("controller_gains/aguiar/k_pos", k_pos);
    nh_p_.getParam("controller_gains/aguiar/k_currents", k_currents);

    /* Assign the new controller */
    this->pf_algorithm_ =
      new Aguiar(delta, kk, kz, k_pos, k_currents, this->publishers_[0],
          this->publishers_[1], this->publishers_[2]);

    pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));
    
    res.success = true;

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PF node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PF controller switched to Aguiar");
  return true;
}

/* Service to switch to the Brevik Path Following method */
bool PathFollowingNode::SetBrevikService(path_following::SetPF::Request &req,
    path_following::SetPF::Response &res) {

  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PF is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

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
  
  /* Read the gains for the controller */
  double delta_h;
  nh_p_.getParam("controller_gains/brevik/delta_h", delta_h);
 
  try {

    /* Assign the new controller */
    this->pf_algorithm_ = new Brevik(this->publishers_[0], 
        this->publishers_[1], this->publishers_[2], delta_h);
    res.success = true;
    pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PF node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PF controller switched to Brevik");
  return true;
}

/* Service to switch to the Fossen Path Following method */
bool PathFollowingNode::SetFossenService(path_following::SetPF::Request &req,
    path_following::SetPF::Response &res) {

  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PF is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Get the topic names for the publishers */
  std::string surge_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string yaw_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/yaw"); 
  std::string pfollowing_debug_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pfollowing_debug");

  /* Create the publishers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(yaw_topic, 1));
   
  try {

    /* Assign the new controller */
    this->pf_algorithm_ = new Fossen(this->publishers_[0], this->publishers_[1], this->set_path_mode_client_);
    res.success = true;
    pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PF node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PF controller switched to Fossen. This algorithm uses the path closest point to make the computations");
  return true;
}

/* Service to switch to the Romulo Path Following method */
bool PathFollowingNode::SetRomuloService(
    path_following::SetPF::Request &req, path_following::SetPF::Response &res) {

  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PF is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Get the topic names for the publishers */
  std::string surge_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string sway_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/sway");
  std::string rabbit_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/rabbit");
  std::string pfollowing_debug_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pfollowing_debug");

  /* Create the publishers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(sway_topic, 1));
  this->publishers_.push_back(
      nh_.advertise<std_msgs::Float64>(rabbit_topic, 1));

  /* Variables to store the gains of the controller */
  std::vector<double> controller_gains;
  double kz;

  try {

    /* Read the gains for the controller */
    nh_p_.getParam("controller_gains/romulo/ke", controller_gains);
    nh_p_.getParam("controller_gains/romulo/kz", kz);
    controller_gains.push_back(kz);

    /* Assign the new controller */
    this->pf_algorithm_ =
      new Romulo(controller_gains, this->publishers_[0],
          this->publishers_[1], this->publishers_[2]);
    res.success = true;
    pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PF node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PF controller switched to Romulo");
  return true;
}

/* Service to switch to the Lapierre Path Following method */
bool PathFollowingNode::SetLapierreService(
    path_following::SetPF::Request &req, path_following::SetPF::Response &res) {

  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PF is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

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

  /* Variables to store the gains of the controller */
  double k1, k2, k3, theta, k_delta;

  try {

    /* Read the gains for the controller */
    nh_p_.getParam("controller_gains/lapierre/k1", k1);
    nh_p_.getParam("controller_gains/lapierre/k2", k2);
    nh_p_.getParam("controller_gains/lapierre/k3", k3);
    nh_p_.getParam("controller_gains/lapierre/theta", theta);
    nh_p_.getParam("controller_gains/lapierre/k_delta", k_delta);

    /* Assign the new controller */
    this->pf_algorithm_ = new Lapierre(k1, k2, k3, theta, k_delta, this->publishers_[0], this->publishers_[1], this->publishers_[2]);
    pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));
    res.success = true;

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PF node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PF controller switched to Lapierre");
  return true;
}

/* Service to switch to the Pramod Path Following method */
bool PathFollowingNode::SetPramodService(path_following::SetPF::Request &req,
    path_following::SetPF::Response &res) {

  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PF is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  std::string surge_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string yaw_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/yaw");
  std::string pfollowing_debug_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pfollowing_debug");

  /* Create the publishers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(yaw_topic, 1));

  /* Variables to store the gains of the controller */
  double kp, ki;
  std::vector<double> controller_gains;

  try {

    /* Read the gains for the controller */
    nh_p_.getParam("controller_gains/pramod/kp", kp);
    nh_p_.getParam("controller_gains/pramod/ki", ki);
    controller_gains.push_back(kp);
    controller_gains.push_back(ki);

    /* Assign the new controller */
    this->pf_algorithm_ = new Pramod(controller_gains, this->publishers_[0],
        this->publishers_[1], this->set_path_mode_client_);
    pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));
    res.success = true;

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PF node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PF controller switched to Pramod. This algorithm uses the path closest point to make the computations");
  return true;
}

/* Service to switch to the Samson Path Following method */
bool PathFollowingNode::SetSamsonService(path_following::SetPF::Request &req,
    path_following::SetPF::Response &res) {

  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PF is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  std::string surge_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string yaw_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/yaw_rate");
  std::string pfollowing_debug_topic = FarolGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pfollowing_debug");

  /* Create the publishers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(yaw_topic, 1));

  /* Variables to store the gains of the controller */
  double k1, k2, k3, theta, k_delta;

  try {

    /* Read the gains for the controller */
    nh_p_.getParam("controller_gains/samson/k1", k1);
    nh_p_.getParam("controller_gains/samson/k2", k2);
    nh_p_.getParam("controller_gains/samson/k3", k3);
    nh_p_.getParam("controller_gains/samson/theta", theta);
    nh_p_.getParam("controller_gains/samson/k_delta", k_delta);

    /* Assign the new controller */
    this->pf_algorithm_ = new Samson(k1, k2, k3, theta, k_delta, this->publishers_[0], this->publishers_[1], this->set_path_mode_client_);
    pf_algorithm_->setPFollowingDebugPublisher(nh_p_.advertise<farol_msgs::mPFollowingDebug>(pfollowing_debug_topic,1));
    res.success = true;

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PF node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PF controller switched to Samson. This algorithm uses the path closest point to make the computations");
  return true;
}

