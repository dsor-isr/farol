#include "CPFControl.h"
#include <stdexcept>

/* Constructor for the CPF Control law */
CPFControl::CPFControl(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &adjency_matrix, unsigned int vehicle_ID) : adjency_matrix_(adjency_matrix), vehicle_id_(vehicle_ID) {

  /* Check if the adjency matrix is a square matrix. If not, throw an exception */
  if((adjency_matrix.rows() != adjency_matrix.cols()) || adjency_matrix.rows() <= 1) {
    throw std::invalid_argument("Adjency Matrix must be square and must be at least 2x2.");
  }

  /* Check if the current vehicle ID is within the size of the adjency matrix */
  if(vehicle_ID < 0 || vehicle_ID >= adjency_matrix.rows()) {
    throw std::invalid_argument("ID of the vehicle must be between [0, Adjency Matriz Size[.");
  }

  /* Update the number of vehicles in the network */
  this->num_vehicles_ = adjency_matrix.rows();

  /* Update the neighbors in the newtork */
  this->neighbors_ = adjency_matrix.row(vehicle_ID);

}

/* Destructor for the abstract class */
CPFControl::~CPFControl() {}

/**
 * @brief  Method to get the Adjency Matrix that represents the network topology
 *
 * @return  An eigen adjecy matrix of doubles 
 */
Eigen::MatrixXi CPFControl::getAdjencyMatrix() {
  return this->adjency_matrix_;
}

/**
 * @brief  Method to get a vector with the neighbors of the current vehicle
 *
 * @return  An eigen vector with ints
 */
Eigen::VectorXi CPFControl::getNeighbors() {
  return this->neighbors_;
}

/**
 * @brief  Method to update the Adjency Matrix that represents the network topology
 *
 * @return  A boolean informing if update was done successfully or not
 */
bool CPFControl::updateAdjencyMatrix(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &adjency_matrix) {

  /* Deal with the special case where the matrix received is not square */
  if(adjency_matrix.rows() != adjency_matrix.cols()) return false;

  /* Deal with the fact that the matrix received might have a different size than the current one */
  if(adjency_matrix.rows() != this->adjency_matrix_.rows()) return false;

  /* If non of the above conditions appear, just update the new adjency matrix */
  this->adjency_matrix_ = adjency_matrix;
  
  /* Update the neighbors in the newtork */
  this->neighbors_ = adjency_matrix.row(vehicle_id_);

  return true;
}

/**
 * @brief  Method to get the number of vehicles used in the network
 *
 * @return  An int with the number of vehicles in the network
 */
unsigned int CPFControl::getNetworkSize() {
  return this->num_vehicles_;
}

/**
 * @brief  Method to get the current vehicle ID in the network
 *
 * @return  An int with the current vehicle ID
 */
unsigned int CPFControl::getCurrentVehicleID() {
  return this->vehicle_id_;
}

