#include "CpfNode.h"

/**
 * @brief  Method to initialize all the services
 */
void CpfNode::initializeServices() {
  ROS_INFO("Initializing Services for CpfNode");

  /* Get the service names for starting and stoping the path following */
  std::string start_cpf_name = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/start_cpf", "/start_cpf");
  std::string stop_cpf_name = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/stop_cpf", "/stop_cpf");
  std::string change_topology_name = FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/change_topology", "/change_topology");

  /* Advertise the services with these names */
  this->startCPF_srv_= this->nh_.advertiseService(start_cpf_name, &CpfNode::StartService, this);
  this->stopCPF_srv_ = this->nh_.advertiseService(stop_cpf_name, &CpfNode::StopService, this);
  this->change_topology_srv_ = this->nh_.advertiseService(change_topology_name, &CpfNode::ChangeTopologyService, this);
}

/* Start Service callback */
bool CpfNode::StartService(cpf_control::StartStop::Request &req, cpf_control::StartStop::Response &res) {

  /* Check if timer was already running or not */
  if (timer_.hasStarted()) {
    ROS_INFO("CPF is already running");
    return true;
  }

  /* Check if the pointer is not null and start the callback timer */
  if(this->cooperative_ != nullptr) {
    ROS_INFO("CPF will start.");
    timer_.start();
    res.success = true;
  } else {
    ROS_ERROR("For some reason the CPF algorithm is not instantiated. Restart this node");
    timer_.stop();
    res.success = false;
  }

  return true;
}

/* Stop Service callback */
bool CpfNode::StopService(cpf_control::StartStop::Request &req, cpf_control::StartStop::Response &res) {

  /* Check if the timer was running or not */
  if (timer_.hasStarted()) {
    ROS_INFO("CPF will stop.");
  } else {
    ROS_INFO("CPF was not running.");
  }

  /* Call the method that stops the CPF algorithm and resets it */
  this->stop();
  
  res.success = true;
  return true;
}

/* Service to change the topology of the network */
bool CpfNode::ChangeTopologyService(cpf_control::ChangeTopology::Request &req, cpf_control::ChangeTopology::Response &res) {

  int new_matrix_size = req.adjency_matrix.size();

  /* Check the size of the received adjency_matrix in the form of an std::vector */
  if (new_matrix_size != this->adjency_matrix_.rows() * this->adjency_matrix_.cols()) {
    
    /* In this case do not update to the new matrix and return */
    ROS_INFO("New Matrix does not have the same size as the current Adjency Matrix");
    res.success = false;
    return true;
  }

  /* Otherwise update the new topology */
  for(int i = 0; i < this->adjency_matrix_.rows(); i++) {
    for (int j = 0; j < this->adjency_matrix_.cols(); j++) {
      this->adjency_matrix_(i, j) = req.adjency_matrix[(i * this->adjency_matrix_.rows()) + j];
    }
  }

  /* Update the value inside the CPF control class */
  this->cooperative_->updateAdjencyMatrix(this->adjency_matrix_);

  /* Inform that everything went smoothly */
  ROS_INFO("Updated new Adjency Matrix successfully!");
  res.success = true;
  return true;
}

