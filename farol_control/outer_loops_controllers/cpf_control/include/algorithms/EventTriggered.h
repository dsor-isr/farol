#pragma once

#include "CPFControl.h"
#include <vector>
#include <array>

/** 
 *  @brief     Auxiliar structure to hold information regarding one vehicle
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
typedef struct {

  /** 
   * @brief The real value of gamma received by the network and the corresponding time
   * at which this value was received by the network 
   */
  double gamma{0.0};

  /**
   * @brief The time instant expressed in seconds corresponding to the instant
   * when gamma was received
   */
  double time{0.0};

  /**
   * @brief The estimated value of gamma 
   */
  double gamma_hat{0.0};

  /** 
   * @brief The desired speed for that gamma 
   */
  double vd{0.0};

  /** 
   * @brief Flag that will become true when the first value is received from 
   * the network for this vehicle 
   */
  bool is_active{false};

} VehicleInfo;

/** 
 *  @brief     Implements a CPF controller using Event triggered communications
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright GPLv3
 */
class EventTriggered: public CPFControl {

  public:
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
    EventTriggered(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &adjency_matrix, unsigned int vehicle_ID, double k_epsilon, double c0, double c1, double alpha);

    /**
     * @brief  The destructor for the EventTriggered class
     */
    ~EventTriggered();

    /**
     * @brief  Method to that updated the coordination control law
     * and returns the correction speed vc to be used by the virtual target
     *
     * @param time  An int with the current time expressed in seconds
     *
     * @return  A double with the speed correction term vc
     */
    double coordinationController(double time) override; 

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
    bool updateVehiclesInformation(double time, unsigned int ID, double gamma, double vd) override;

    /**
     * @brief  Method to inform the user if the current gamma should be published or not
     * This method will update the value of gamma last sent to the network when this method returns true
     *
     * @param time  The current time expressed in seconds
     *
     * @return A boolean that is true if the current gamma should be published
     */
    bool publishCurrentGamma(double time) override;

    /**
     * @brief  Method to reset the current CPF controller
     *
     * @return  A boolean whether it was reset correctly or not
     */
    bool reset() override;

  protected:

    /**
     * @brief  Method to estimate the current gamma of a vehicle represented by ID
     *
     * @param ID  The ID of the vehicle to update the prediction
     * @param time  The current time expressed in seconds
     */
    void predictVehicleEstimator(unsigned int ID, double time); 

  private:

    /**
     * @brief The gain for the correction term 
     */
    double k_epsilon_{1.0};

    /** 
     * @brief The maximum error admited before emiting a gamma for the network 
     */
    double c0_{1.0};
    double c1_{1.0};
    double alpha_{1.0};

    /** 
     * @brief Last Gamma that was broadcast to the multi-vehicle network 
     */
    double time_last_broadcast_{0.0};
    double last_gamma_broadcast_{0.0};
    bool first_broadcast_{true};
    double time_first_broadcast_{0.0};

    /** 
     * @brief Array to store the estimated values of the gamma for the vehicles in the network 
     */
    VehicleInfo * vehicles_data_;
};
