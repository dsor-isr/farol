#include "CpfNode.h"

/**
 * @brief  Constructor for the CpfNode
 *
 * @param nh  Pointer to the public nodehandle
 * @param nh  Pointer to the private nodehandle
 */
CpfNode::CpfNode(ros::NodeHandle * nh, ros::NodeHandle * nh_p):nh_(*nh), nh_p_(*nh_p) {
  ROS_INFO("in class constructor of CpfNode");

  /* Initialize the default controller */
  this->cooperative_ = this->createEventTriggeredControl();

  /* Initialize the ROS part */
  this->initializeSubscribers();
  this->initializePublishers();
  this->initializeServices();
  this->initializeTimer();
}

/**
 * @brief  Destructor for the CpfNode
 */
CpfNode::~CpfNode() {

  /* Shutdown the subscribers */
  this->external_gamma_sub_.shutdown();
  this->internal_gamma_sub_.shutdown();

  /* Shutdown the publishers */
  this->vc_pub_.shutdown();
  this->cpf_server_pub_.shutdown();

  /* Free the memory allocated for the cooperative object */
  if(this->cooperative_) {
    delete this->cooperative_;
  }

  /* Shutdown the timer */
  timer_.stop();

  /* Shutdown the node */
  nh_.shutdown();
}

CPFControl* CpfNode::createEventTriggeredControl() {
  /* Read the network and control parameters */
  int ID;
  std::vector<int> adj_matrix;
  double k_epsilon, c0, c1, alpha;

  /* Read the network and control parameters */
  nh_p_.getParam("ID", ID);
  nh_p_.getParam("adjency_matrix", adj_matrix);
  nh_p_.getParam("gains/event_triggered/c0", c0);
  nh_p_.getParam("gains/event_triggered/c1", c1);
  nh_p_.getParam("gains/event_triggered/alpha", alpha);
  nh_p_.getParam("gains/event_triggered/k_epsilon", k_epsilon);
  
  /* Save the ID in the node */
  this->ID_ = ID;

  /* Compute the sqrt of the size of the adjency matrix to check if it is square */
  int num_vehicles = static_cast<int>(std::sqrt(adj_matrix.size()));

  if(std::pow(num_vehicles, 2) != adj_matrix.size()) {
    throw std::invalid_argument("The adjency Matrix in the configuration file in not square!");
  }

  /* Generate an Eigen Matrix from the vector that contains the adjency matrix */
  this->adjency_matrix_.resize(num_vehicles, num_vehicles);

  for(int i = 0; i < num_vehicles; i++) {
    for (int j = 0; j < num_vehicles; j++) {
      this->adjency_matrix_(i, j) = adj_matrix[(i * num_vehicles) + j];
    }
  }

  /* Allocate a cooperative path following control node */
  return new EventTriggered(this->adjency_matrix_, ID, k_epsilon, c0, c1, alpha); 
}

/**
 * @brief  Method to stop the current path following algorithm
 */
bool CpfNode::stop() {
  
  /* Stop the timer */
  this->timer_.stop();

  /* Reset the auxiliary variables */
  this->gamma_ = 0.0;
  this->vd_ = 0.0;
  this->seq_ = 0;

  /* Call the reset method in the CPF algorithm */
  if (this->cooperative_) this->cooperative_->reset();

  return true;
}

/**
 * @brief  Method to initialize all the subscribers 
 */
void CpfNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for CpfNode");

  /* Get the topic names for the subscribers */
  std::string gamma_topic = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/subscribers/internal_gamma");
  std::string external_gamma_topic = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/subscribers/external_gamma");

  /* Initialize the subscribers */
  this->internal_gamma_sub_ = nh_.subscribe(gamma_topic, 10, &CpfNode::internalInfoCallback, this);
  this->external_gamma_sub_ = nh_.subscribe(external_gamma_topic, 10, &CpfNode::externalInfoCallback, this);
}

/**
 * @brief  Method to initialize all the publishers 
 */
void CpfNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for CpfNode");

  /* Get the topic names for the publishers */
  std::string vc_topic = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/publishers/vc");
  std::string cpf_server_input_topic = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/publishers/cpf_server_input");

  /* Initialize the publishers */
  this->vc_pub_ = nh_.advertise<std_msgs::Float64>(vc_topic, 1);
  this->cpf_server_pub_ = nh_.advertise<farol_msgs::CPFGamma>(cpf_server_input_topic, 1);

}

/**
 * @brief  Method to create the timer that will do all the work 
 */
void CpfNode::initializeTimer() {
  this->timer_ =nh_.createTimer(ros::Duration(1.0 / CpfNode::nodeFrequency()), &CpfNode::timerIterCallback, this);
  this->timer_.stop();
}

/**
 * @brief  Method to retrieve the working frequency of this node 
 *
 * @return  A double with the frequency of the node
 */
double CpfNode::nodeFrequency() {
  double node_frequency;
  this->nh_.param("node_frequency", node_frequency, 2.0);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

/**
 * @brief  Callback for the timer interruption. Where all the logic of the algorithms
 * is executed with a fixed period
 *
 * @param event  The timer event (unused)
 */
void CpfNode::timerIterCallback(const ros::TimerEvent &event) {

  double t = ros::Time::now().toSec();

  /* Run the coordination controller */
  double vc = this->cooperative_->coordinationController(t);

  /* Publish the correction factor to the VC topic */
  std_msgs::Float64 msg;
  msg.data = vc;
  this->vc_pub_.publish(msg);

  /* Check if it is time to publish the current gamma to the vehicle network */
  bool pub = this->cooperative_->publishCurrentGamma(t);

  if(pub) {

    /* publish the current gamma to the network */
    farol_msgs::CPFGamma msg;

    msg.header.seq = this->seq_;
    this->seq_++;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";

    msg.ID = this->cooperative_->getCurrentVehicleID();
    msg.gamma = this->gamma_; 
    msg.vd = this->vd_;

    this->cpf_server_pub_.publish(msg);
  }
}

/* Callback for receiving the external vehicle data */
void CpfNode::externalInfoCallback(const farol_msgs::CPFGamma& msg) {

  double t = ros::Time::now().toSec();

  /* Interpret the message */
  unsigned int vehicle_ID = msg.ID;
  double gamma = msg.gamma;
  double vd = msg.vd;

  /* Ignore an external message that contains data respective to ou vehicle */
  if(vehicle_ID == this->ID_) return;

  /* Update the data inside the cooperative library */
  if(this->cooperative_) {
    this->cooperative_->updateVehiclesInformation(t, vehicle_ID, gamma, vd);
  }

}

/* Callback for receiving the data from this vehicle */
void CpfNode::internalInfoCallback(const dsor_paths::PathData &msg) {

  double t = ros::Time::now().toSec();

  /* Update the temporary varibales */
  this->gamma_ = msg.gamma;
  this->vd_ = msg.vd; 

  /* Update the data inside the cooperative library */
  if(this->cooperative_) {
    this->cooperative_->updateVehiclesInformation(t, this->ID_, this->gamma_, this->vd_);
  }
}

/**
 * @brief  The main function. THe entry point for this ros node
 *
 * @param argc  The number of arguments
 * @param argv  The array of arguments
 *
 * @return   An integer with success or failure
 */
int main(int argc, char** argv) {

  /* Inititate the node and the node handles*/
  ros::init(argc, argv, "cpf_node"); 
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  ROS_INFO("main: instantiating an object of type CpfNode");

  /* Instantiate the object and go into spin. Let the timer callback work */
  CpfNode cpfNode(&nh, &nh_p);
  ros::spin();

  return 0;
}

