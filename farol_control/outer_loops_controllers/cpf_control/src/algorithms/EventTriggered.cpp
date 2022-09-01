#include "EventTriggered.h"
#include <cmath>

/**
 * @brief  Constructor for the EventTriggered CPF class. Receive the adjency matrix
 * as a parameter
 *
 * @param adjency_matrix  An Eigen adjency matrix
 * @param vehicle_ID  An unsigned int with the ID of this particular vehicle
 * @param k_epsilon  The gain for the correction control law
 * @param c0 The gain for the event trigger threshold
 * @param c1 The gain for the event trigger threshold
 * @param alpha The gian for the event trigger threshold
 */
EventTriggered::EventTriggered(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &adjency_matrix, 
                                unsigned int vehicle_ID, 
                                double k_epsilon, 
                                double c0,
                                double c1,
                                double alpha) : 
  CPFControl(adjency_matrix, vehicle_ID),
    k_epsilon_(k_epsilon),
    c0_(c0),
    c1_(c1),
    alpha_(alpha) {

  /* Alocate memory for the array of estimated gammas */
  this->vehicles_data_ = new VehicleInfo[this->getNetworkSize()];
}

/**
 * @brief  Destructor for the EventTriggered class
 */
EventTriggered::~EventTriggered() {

  /* Free the memory used for the estimated gammas */
  delete[] this->vehicles_data_;
}

/**
 * @brief  Method to that updated the coordination control law
 * and returns the correction speed vc to be used by the virtual target
 *
 * @param time  An int with the current time expressed in seconds
 *
 * @return  A double with the speed correction term vc
 */
double EventTriggered::coordinationController(double time) {
  
  double gamma_hat_sum = 0.0;
  double sync_error = 0.0;
  int num_vehicles_used = 0;
  double vc = 0.0;

  /* Get the line of the adjency matrix that tells who talks to this vehicle */
  Eigen::VectorXi neighbors_vec = this->getNeighbors();

  /* If we do not have data on our on vehicle yet, PF has not started yet */
  if(!this->vehicles_data_[this->getCurrentVehicleID()].is_active) return 0.0;

  /* For all the vehicles in the network */
  for(unsigned int id = 0; id < this->getNetworkSize(); id++) {
  
    /* If the vehicle i is a neighbor of this particular vehicle */
    if(neighbors_vec[id] == 1 && this->vehicles_data_[id].is_active && id != this->getCurrentVehicleID()) {
        
        /* Predict virtual target of the neighbors in the current time */
        this->predictVehicleEstimator(id, time);
    
        /* Acumulate the sum of the gamma_hat of the neighbors */
        gamma_hat_sum += this->vehicles_data_[id].gamma_hat;
        num_vehicles_used++;
    }
  }

  /* If we already have data from the other vehicles */ 
  if(num_vehicles_used > 0) {
    
    /* Compute the synchronization law */
    sync_error = this->vehicles_data_[this->getCurrentVehicleID()].gamma - ((1.0 / num_vehicles_used) * gamma_hat_sum);
   
    /* Compute the correction speed and saturate using hyperbolic tangent */
    vc = -this->k_epsilon_ * tanh(sync_error);
  }

  return vc;
}

/**
 * @brief  Method to update each individual vehicle information
 * given an int with the ID of the vehicle and the new virtual target value
 *
 * @param time  An int with the current time expressed in seconds
 * @param ID  An int with the ID of the vehicle which to update information
 * @param gamma  A double with the new information of the virtual target position
 * @param vd  A double with the desired velocity of the virtual target of that particular vehicle
 *
 * @return A boolean with the information whether the information was used with success or not
 */
bool EventTriggered::updateVehiclesInformation(double time, unsigned int ID, double gamma, double vd) {
  
  /* First check if ID is whithin the valid bounds */
  if(ID < 0 || ID >= this->getNetworkSize()) return false;

  /* Update the gamma in the corresponding vehicle */
  this->vehicles_data_[ID].gamma = gamma;
  this->vehicles_data_[ID].time = time;
  this->vehicles_data_[ID].vd = vd;
  this->vehicles_data_[ID].is_active = true;

  return true;
}

/**
 * @brief  Method to inform the user if the current gamma should be published or not
 *
 * @param time The current time expressed in seconds
 *
 * @return A boolean that is true if the current gamma should be published
 */
bool EventTriggered::publishCurrentGamma(double time) {
    
  /* Get the ID of our vehicle */
  unsigned int ID = this->getCurrentVehicleID();

  /* First check if we have already received values from gamma */
  if(!this->vehicles_data_[ID].is_active) return false;

  /* If gamma was not published yet, then publish it (first iteration) */
  if(this->first_broadcast_) {
    this->first_broadcast_ = false;
    this->last_gamma_broadcast_ = this->vehicles_data_[ID].gamma;
    this->time_last_broadcast_ = time;
    this->time_first_broadcast_ = time;
    return true;
  } 

  /* Compute the estimator that the other vehicles see 
   * Note, here we do not want to compute gamma hat the standard way, because
   * the gamma that counts is the last that went to the network (that is the 
   * one we should compute the estimation with) and not with the lastest gamma
   * that we got internally
   */

  double my_gamma_hat = this->last_gamma_broadcast_ + (this->vehicles_data_[ID].vd * (time - this->time_last_broadcast_));

  /* Compute the error between the gamma of this real vehicle and the predicted gamma for this real vehicle */
  double gamma_error = std::abs(my_gamma_hat - this->vehicles_data_[ID].gamma);
  double t = std::abs(this->time_first_broadcast_ - time);
  double threshold = this->c0_ + (this->c1_ * std::exp(-this->alpha_ * t)); 

  /* Only publish the gamma to the network is the error is above a given threshold */
  if(gamma_error > threshold) {

    /* Update the last information sent to the network */
    this->last_gamma_broadcast_ = this->vehicles_data_[ID].gamma;
    this->time_last_broadcast_ = time;
    return true;
  }

  /* Otherwise, do not publish in this iteration */
  return false;
}

/**
 * @brief  Method to estimate the current gamma of a vehicle represented by ID
 *
 *
 * @param ID  The ID of the vehicle to update the prediction
 * @param time  The current time expressed in seconds
 */
void EventTriggered::predictVehicleEstimator(unsigned int ID, double time) {
  
  /* First check if ID is whithin the valid bounds */ 
  if(ID < 0 || ID >= this->getNetworkSize()) return;

  /* Check if the time stamp received is lower than the previous time */
  if(time < this->vehicles_data_[ID].time) return;

  /* Update the prediction (gamma_hat) for that particular vehicle */
  if(this->vehicles_data_[ID].is_active) { 
    this->vehicles_data_[ID].gamma_hat = this->vehicles_data_[ID].gamma + this->vehicles_data_[ID].vd * (time - this->vehicles_data_[ID].time);
  }

  return;
} 

/**
 * @brief  Method to reset the current CPF controller
 *
 * @return  A boolean whether it was reset correctly or not
 */
bool EventTriggered::reset() {

  /* Reset all the variables */
  this->time_last_broadcast_ = 0.0;
  this->last_gamma_broadcast_ = 0.0;
  this->first_broadcast_ = true;
  this->time_first_broadcast_ = 0.0;

  /* Free the old memory used to store the vehicle data */
  delete[] this->vehicles_data_;

  /* Alocate memory for the array of estimated gammas */
  this->vehicles_data_ = new VehicleInfo[this->getNetworkSize()];

  /* Inform the reset was made with success*/
  return true;
}

