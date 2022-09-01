#pragma once

#include <Eigen/Core>
#include <limits>

/**
 * @brief A structure to hold the state of the vehicle
 * @author    Marcelo Jacinto
 * @author    Joao Quintas
 * @author    Joao Cruz
 * @author    Hung Tuan
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
typedef struct {

  /**
   * @brief Velocities in the body frame
   */
  Eigen::Vector3d v1{0.0, 0.0, 0.0};    // Linear velocities in the body frame
  Eigen::Vector3d v2{0.0, 0.0, 0.0};    // Angular velocities in the body frame

  /**
   * @brief Positions and orientations in inertial frame
   */
  Eigen::Vector3d eta1{0.0, 0.0, 0.0};  // Position relative to the inertial frame
  Eigen::Vector3d eta2{0.0, 0.0, 0.0} ; // Orientation relative to the inertial frame

 } VehicleState;


/**
 * @brief  A structure to hold the data of the path
 * @author    Marcelo Jacinto
 * @author    Joao Quintas
 * @author    Joao Cruz
 * @author    Hung Tuan
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
typedef struct {
  
  /**
   * @brief The value of gamma used in the computations for this values 
   */
  double gamma{0.0};
  
  /**
   * @brief Desired position and its derivatives 
   */
  Eigen::Vector3d pd{0.0, 0.0, 0.0};
  Eigen::Vector3d d_pd{0.0, 0.0, 0.0};
  Eigen::Vector3d dd_pd{0.0, 0.0, 0.0};
  
  /**
   * @brief Other properties of the path such as the psi, curvature and tangent_norm 
   */
  double psi{0.0};
  double curvature{0.0};
  double tangent_norm{0.0};

  /**
   * @brief The desired speed for a given gamma
   */
  double vd{0.0};
  double d_vd{0.0};
  double vehicle_speed{0.0};

  /**
   * @brief The correction speed for gamma (when cooperating with other vehicles 
   */
  double vc{0.0};

  /**
   * @brief Properties related to the boundaries of the parameterization of the path 
   */
  double gamma_min{std::numeric_limits<double>::lowest()};
  double gamma_max{std::numeric_limits<double>::max()};

} PathState;

typedef struct {

  std::string algorithm;
  double cross_track_error{0.0};
  double along_track_error{0.0};
  double yaw{0.0};
  double psi{0.0};
  double gamma{0.0};

} PFollowingDebug;

