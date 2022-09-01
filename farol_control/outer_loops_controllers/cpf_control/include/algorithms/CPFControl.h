#pragma once

#include <Eigen/Core>

/** 
 *  @brief     Abstract class to implement the cooperative path following 
 *             synchronization controller
 *
 *             In this class it is assumed that each vehicle has an ID which 
 *             corresponds to each position in the adjency matrix that describes 
 *             the topology of the network. The first ID is assumed to be 0. 
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class CPFControl {
  
  public:
    
    /**
     * @brief  Method to that updated the coordination control law
     * and returns the correction speed vc to be used by the virtual target
     *
     * @param time  An int with the current time expressed in seconds
     *
     * @return  A double with the speed correction term vc
     */
    virtual double coordinationController(double time) = 0; 
    
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
    virtual bool updateVehiclesInformation(double time, unsigned int ID, double gamma, double vd) = 0;

    /**
     * @brief  Method to inform the user if the current gamma should be published or not
     *
     * @param time  The current time expressed in seconds
     *
     * @return A boolean that is true if the current gamma should be published
     */
    virtual bool publishCurrentGamma(double time) = 0;

    /**
     * @brief  Method to reset the current CPF controller
     *
     * @return  A boolean whether it was reset correctly or not
     */
    virtual bool reset() = 0;

    /**
     * @brief  Method to get the Adjency Matrix that represents the network topology
     *
     * @return  An eigen adjecy matrix of ints
     */
    Eigen::MatrixXi getAdjencyMatrix();

    /**
     * @brief  Method to get a vector with the neighbors of the current vehicle
     *
     * @return  An eigen vector with ints
     */
    Eigen::VectorXi getNeighbors();

    /**
     * @brief  Method to update the Adjency Matrix that represents the network topology
     *
     * @return  A boolean informing if update was done successfully or not
     */
    bool updateAdjencyMatrix(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &adjency_matrix);

    /**
     * @brief  Method to get the number of vehicles used in the network
     *
     * @return  An int with the number of vehicles in the network
     */
    unsigned int getNetworkSize();

    /**
     * @brief  Method to get the current vehicle ID in the network
     *
     * @return  An int with the current vehicle ID
     */
    unsigned int getCurrentVehicleID();
    
    /**
     * @brief  Virtual destructor for the abstract class
     */
    virtual ~CPFControl();
  
  protected:

    /**
     * @brief  Constructor for the abstract class. Receive the adjency matrix
     * as a parameter
     *
     * @param adjency_matrix  An Eigen adjency matrix
     * @param vehicle_ID  The current vehicle ID
     */
    CPFControl(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &adjency_matrix, unsigned int vehicle_ID);
    
  private:

    /**
     * @brief Stores the adjency matrix with the topology of the network 
     */
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> adjency_matrix_;
    
    /** 
     * @brief Stores the neighbours of the current vehicle 
     */
    Eigen::VectorXi neighbors_;

    /** 
     * @brief Stores the size of the network 
     */
    unsigned int num_vehicles_;

    /** 
     * @brief Stores the current vehicle ID 
     */
    unsigned int vehicle_id_;
};
