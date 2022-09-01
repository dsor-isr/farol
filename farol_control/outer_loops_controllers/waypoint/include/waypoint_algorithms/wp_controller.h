/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
*/
#ifndef CATKIN_WS_WAYPOINTALGORITHM_H
#define CATKIN_WS_WAYPOINTALGORITHM_H

// some generically useful stuff to include...
#include <iostream>
#include <math.h>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <ros/ros.h> //ALWAYS need to include this
#include <std_msgs/Float64.h>

/**
 * @brief  Vehicle state struct. Contains position, orientation and linear and
 * angular velocities
 */
struct Vehicle_t {
  Eigen::Vector3d eta1{0.0, 0.0, 0.0}; // vehicle position vector
  Eigen::Vector3d eta2{0.0, 0.0, 0.0}; // vehicle orientation vector
  Eigen::Vector3d v1{0.0, 0.0, 0.0};   // linear velocities
  Eigen::Vector3d v2{0.0, 0.0, 0.0};   // angular velocities
};

/**
 * @brief  Waypoint reference struct. Contains references for poistion and
 * orientations. Not everything needs to be used.
 */
struct WPref_t {
  Eigen::Vector3d eta1{0.0, 0.0, 0.0}; // vehicle position vector (output)
  Eigen::Vector3d eta2{0.0, 0.0, 0.0}; // vehicle orientation vector (output)
};

/**
 * @brief  Waypoint output struct. Contains orientation and linear and angular
 * velocities. Not everything needs to be used.
 */
struct Output_t {
  Eigen::Vector3d eta2{0.0, 0.0, 0.0}; // orientation vector (desired)
  Eigen::Vector3d v1{0.0, 0.0, 0.0};   // linear velocities (desired)
  Eigen::Vector3d v2{0.0, 0.0, 0.0};   // angular velocities (desired)
};

/**
 * @brief  Abstract class of a waypoint controller
 */
class WaypointController {
public:
  WaypointController() {}

  virtual ~WaypointController() {}

  /**
   * @brief  Mutator for setting the controller period
   *
   * @param f frequency of the main loop
   */
  void setFrequency(const double &f) { ts_ = 1 / f; }

  /**
   * @brief  Mutator for updating the gains
   *
   * @param gains parameters of the waypoiny controller
   */
  void setGains(const std::vector<double> &gains) { gains_ = gains; };

  /**
   * @brief  Computes and publishes the output of the controller
   *
   * @param state
   * @param wp_ref
   */
  void compute(Vehicle_t state, WPref_t wp_ref) {
    calculateRef(state, wp_ref);
    publish();
  }

protected:
  double ts_{0.0};
  std::vector<double> gains_;

  /**
   * @brief  Virtual function to publish the output of the controller
   */
  virtual void publish() = 0;

  /**
   * @brief  Virtual function to compute the waypoint controller
   *
   * @param state
   * @param wp_ref
   */
  virtual void calculateRef(Vehicle_t state, WPref_t wp_ref) = 0;

  /**
   * @brief  Getter of the yaw reference (output of controller)
   *
   * @return  yaw ref
   */
  double getYawOut() { return wp_out_.eta2[2]; }

  /**
   * @brief  Getter of the yaw rate reference (output of controller)
   *
   * @return  yaw rate ref
   */
  double getYawrateOut() { return wp_out_.v2[2]; }

  /**
   * @brief  Getter of the surge reference (output of controller)
   *
   * @return  surge ref
   */
  double getSurgeOut() { return wp_out_.v1[0]; }

  /**
   * @brief  Getter of the sway reference (output of controller)
   *
   * @return  sway ref
   */
  double getSwayOut() { return wp_out_.v1[1]; }

  /**
   * @brief  Yaw reference setter (output of controller)
   *
   * @param value yaw ref
   */
  void setYawOut(const double &value) { wp_out_.eta2[2] = value; }

  /**
   * @brief  Yaw rate reference setter (output of controller)
   *
   * @param value yaw rate ref
   */
  void setYawrateOut(const double &value) { wp_out_.v2[2] = value; }

  /**
   * @brief  Surge reference setter (output of controller)
   *
   * @param value surge ref
   */
  void setSurgeOut(const double &value) { wp_out_.v1[0] = value; }

  /**
   * @brief  Sway reference setter (output of controller)
   *
   * @param value sway ref
   */
  void setSwayOut(const double &value) { wp_out_.v1[1] = value; }

private:
  Output_t wp_out_; // output of the controller
};
#endif // CATKIN_WS_WAYPOINTALGORITHM_H
