#include "PathNode.h"

/**
 * @brief  PathNode constructor. Receives the ROS node handles as inputs and 
 * initializes the subscribers, publishers, timers, parameters, etc...
 *
 * @param nh  A pointer to the public ROS node handle 
 * @param nh_p  A pointer to the private ROS node handle  
 */
PathNode::PathNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p):nh_(*nh), nh_p_(*nh_p){

  /* Instantiate the ROS subscribers, publishers, etc */
  ROS_INFO("in class constructor of PathNode");
  this->loadParams();
  this->initializeSubscribers();
  this->initializePublishers();
  this->initializeServices();
  this->initializeTimer();

  /* Allocate memory to store the Path object */
  this->path_ = new Path();
}

/**
 * @brief  Class destructor. Called when deleting the class object.
 */
PathNode::~PathNode() {

  /* Shutdown all the publishers */
  this->path_pub_.shutdown(); 
  this->virtual_target_pub_.shutdown();

  /* Shutdown all the subscribers */
  this->gamma_sub_.shutdown();
  this->vehicle_sub_.shutdown();

  /*Shutdown all the services */
  this->shutdownServices();

  /* Stop the timer */
  this->timer_.stop();

  /* Free the memory used by the path */
  delete this->path_;

  /* Shutdown the node */
  this->nh_.shutdown();
}

void PathNode::loadParams() {
  ROS_INFO("Loading Parameters for PathNode");

  this->frame_id_ = FarolGimmicks::getParameters<std::string>(this->nh_p_, "frame_id");
}

/**
 * @brief  A method for initializing all the subscribers. This method is called
 * by the constructor of the PathNode class upon object creation
 */
void PathNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for PathNode");

  /* Get the topic names for the subscribers */
  std::string gamma_topic = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/subscribers/gamma");
  std::string vehicle_topic = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/subscribers/vehicle_state");

  /* Initialize the subscribers */
  this->gamma_sub_ = nh_.subscribe(gamma_topic, 10, &PathNode::gammaCallback, this);
  this->vehicle_sub_ = nh_.subscribe(vehicle_topic, 10, &PathNode::vehicleStateCallback, this);
}

/**
 * @brief  A method for initializing all the subscribers. This method is called by
 * the constructor of the PathNode class upon object creation
 */
void PathNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for PathNode"); 	  

  /* Get the topic names for the publishers */
  std::string path_data_topic = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/publishers/path_data");
  std::string virtual_target_topic = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/publishers/virtual_target_state");

  /* Initialize the publishers */
  this->path_pub_ = nh_.advertise<dsor_paths::PathData>(path_data_topic, 1); 
  this->virtual_target_pub_ = nh_.advertise<farol_msgs::mState>(virtual_target_topic, 1);

}

/**
 * @brief  A method for initializing all the timers. This method is called by the
 * constructor of the PathNode class upon object creation
 */
void PathNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0/PathNode::nodeFrequency()), &PathNode::timerIterCallback, this);
}

/**
 * @brief  A method do update the node update rate from the configurations in the ROS
 * parameter server
 *
 * @return  A double with the frequency used
 */
double PathNode::nodeFrequency() {
  double node_frequency = FarolGimmicks::getParameters<double>(this->nh_p_, "node_frequency", 5);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

void PathNode::timerIterCallback(const ros::TimerEvent &event) {

  /* Declare the variables to store the values of the path */
  std::optional<Eigen::Vector3d> pd;
  std::optional<Eigen::Vector3d> d_pd;
  std::optional<Eigen::Vector3d> dd_pd;
  std::optional<double> tangent = 0;
  std::optional<double> curvature = 0;
  std::optional<double> derivative_norm = 0;
  std::optional<double> vd = 0;
  std::optional<double> d_vd = 0;
  double vehicle_speed = 0;
  std::pair<double, double> min_max_gamma_path;

  /* If the mode is to use the closest point to the vehicle, than get the gamma of that point */
  if(this->closer_point_mode_) {
    /* Then just override the gamma with the one corresponding to the closest point on the path */
    this->gamma_ = this->path_->getClosestGamma(this->vehicle_pos_);
  }

  /* Construct the message to send with the path information and with the virtual target current state */
  dsor_paths::PathData msg; 
  farol_msgs::mState vt_state_msg;

  if(!this->path_->isEmpty() && this->gamma_.has_value()) {

    /* Get the position, first and second derivatives */
    pd = this->path_->eq_pd(this->gamma_.value());
    d_pd = this->path_->eq_d_pd(this->gamma_.value());
    dd_pd = this->path_->eq_dd_pd(this->gamma_.value());

    /* Get the values of the tangent angle, curvature and derivative_norm */
    tangent = this->path_->tangent(this->gamma_.value());
    curvature = this->path_->curvature(this->gamma_.value());
    derivative_norm = this->path_->derivative_norm(this->gamma_.value());
    vd = this->path_->eq_vd(this->gamma_.value());
    d_vd = this->path_->eq_d_vd(this->gamma_.value());
    
    /* Compute the vehicle speed */
    vehicle_speed = vd.value_or(0.0) * derivative_norm.value_or(0.0);
  
    /* Get the values for the minimum and maximum values allowed for the path */
    min_max_gamma_path = this->path_->getMinMaxGamma();

    /* Check if we have all the data */
    if(pd && d_pd && dd_pd && tangent && curvature && derivative_norm) {

      /* Header for the message */
      msg.header.seq = this->seq_;
      this->seq_++; //Increment the sequence id
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = this->frame_id_;

      /* The value of gamma used to make the computations */
      msg.gamma = this->gamma_.value();

      /* The data that we got from the path */
      for(int i = 0; i < 3; i++) {
        msg.pd[i] = pd.value()[i];
        msg.d_pd[i] = d_pd.value()[i];
        msg.dd_pd[i] = dd_pd.value()[i]; 
      }

      msg.curvature = curvature.value();
      msg.tangent = tangent.value();
      msg.derivative_norm = derivative_norm.value();

      msg.vd = vd.value();
      msg.d_vd = d_vd.value();
      msg.vehicle_speed = vehicle_speed;

      msg.gamma_min = min_max_gamma_path.first;
      msg.gamma_max = min_max_gamma_path.second;

      /* Publish the message with path info */
      this->path_pub_.publish(msg);

      /* Create a message with the state of the virtual target and 
       * publish it (to be seen as a virtual vehicle)
       */
      vt_state_msg.Y = pd.value()[0];
      vt_state_msg.X = pd.value()[1];
      vt_state_msg.Z = pd.value()[2];
      vt_state_msg.Yaw = tangent.value() * 180.0 / M_PI; 

      /* Publish the message with the state of the virtual target */
      this->virtual_target_pub_.publish(vt_state_msg);
    }
  }
}

/**
 * @brief  Callback to update the current gamma of the path
 *
 * @param msg  A Float64/Double with the current value of gamma
 */
void PathNode::gammaCallback(const std_msgs::Float64 &msg) {

  /* Update the current gamma value */
  this->gamma_ = msg.data;
}

/**
 * @brief  Callback to update the current vehicle position
 *
 * @param msg  A auv_msgs/NavigationStatus messages with the current state of the vehicle
 */
void PathNode::vehicleStateCallback(const auv_msgs::NavigationStatus &msg) {
  
  /* Update the vehicle position */
  this->vehicle_pos_ <<  msg.position.north, msg.position.east, msg.altitude;
}

/**
 * @brief  Main method. The entry point of the PathNode program
 *
 * @param argc  The number of arguments passed to the program
 * @param argv  The vector with the arguments passed to the program
 *
 * @return  An int simbolizing success of failure
 */
int main(int argc, char** argv) {

  /* ROS set-ups */
  ros::init(argc, argv, "dsor_paths_node"); //node name

  /* create a node handle; need to pass this to the class constructor */
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  ROS_INFO("main: instantiating an object of type PathNode");

  /* instantiate an PathNode class object and pass in pointer to nodehandle for constructor to use */
  PathNode dsorPaths(&nh,&nh_p);

  /*Added to work with timer -> going into spin; let the callbacks do all the work */
  ros::spin();

  return 0;
}

