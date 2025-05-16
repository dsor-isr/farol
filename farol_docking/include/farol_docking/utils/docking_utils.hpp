/**
 * @file   docking_utils.hpp
 * @brief  General utility function used for the docking
 * @author Ravi Regalo <ravi.regalo@tecnico.ulisboa.pt>
 * @date   2025-04-25
 * 
 * Description :)
 */
#pragma once

#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <cmath> 
#include <deque>
#include <vector>
#include <bitset>


template<typename T>
struct Stamped {
  T value;
  double stamp;  // Timestamp, usually in seconds (e.g., UNIX time or relative)

  Stamped() = default;

  Stamped(const T& val, double time)
    : value(val), stamp(time) {}

  // Comparison operator for sorting
  bool operator<(const Stamped& other) const {
    return stamp < other.stamp;
  }
};


struct Measurement {
    Stamped<Eigen::VectorXd> data;
    std::string type;

    Measurement() = default;

    Measurement(const Eigen::VectorXd& vec, double stamp, const std::string& tag)
        : data(vec, stamp), type(tag) {}
};

struct Reference {
    Eigen::VectorXd data;
    std::string frame_id;

    Reference() = default;

    Reference(const Eigen::VectorXd& data_, const std::string& frame_id_)
        : data(data_), frame_id(frame_id_) {}
};





// inline Eigen::Vector3d extractRPY(const Eigen::Matrix3d& R) {
//     auto euler = R.eulerAngles(2, 1, 0); // yaw, pitch, roll
//     return {euler[2], euler[1], euler[0]}; // roll, pitch, yaw
// }

  


/**
 * @brief Converts an array in spherical coordinates to cartesian coordinates 
 * 
 *      [range, bearing, elevation] -> [x, y, z]
 *
 * @param[in]  rbe  [range, bearing, elevation] array
 * @return Eigen::Vector3d [x,y,z]
 *
 */
inline Eigen::Vector3d rbe_to_xyz(Eigen::Vector3d rbe) {
    Eigen::Vector3d xyz; 
    xyz(0) = rbe(0) * cos(rbe(2)) * cos(rbe(1));
    xyz(1) = rbe(0) * cos(rbe(2)) * sin(rbe(1));
    xyz(2) = rbe(0) * sin(rbe(2));
    return xyz;
}

inline double radiansToDegrees360(double radians) {
    double degrees = radians * (180.0 / M_PI);  
    if (degrees < 0) {
        degrees += 360.0;  
    }
    return degrees;
}


inline Eigen::Matrix2d Rot2D(double yaw) {
    Eigen::Matrix2d rot;
    rot <<  cos(yaw), -sin(yaw),
            sin(yaw),  cos(yaw);
    return rot;
}


inline double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) 
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

inline double wrapTo2Pi(double angle) {
    angle = fmod(angle, 2.0 * M_PI);
    if (angle < 0) 
        angle += 2.0 * M_PI;
    return angle;
}

inline double sigma_e(double input) {
    if (input > 1)
        return 1;
    else if (input < -1)
        return -1;
    return input;
}


inline Eigen::Vector3d extractRPY(const Sophus::SO3d& R) {
    Eigen::Matrix3d rot = R.matrix();
  double pitch;
  if (std::abs(rot(2, 0)) < 1.0 - 1e-6) {
    pitch = std::asin(-rot(2, 0));
  } else {
    // Gimbal lock (pitch = ±90º)
    pitch = (rot(2, 0) > 0) ? -M_PI_2 : M_PI_2;
  }

  double roll = std::atan2(rot(2, 1), rot(2, 2));
  double yaw  = std::atan2(rot(1, 0), rot(0, 0));

  // Wrap to [-π, π] for consistency
  return Eigen::Vector3d(
    wrapToPi(roll),
    wrapToPi(pitch),
    wrapToPi(yaw)
  );
  }

// /**
//  * @brief Brief one-line description of what the function does.
//  * 
//  * Detailed explanation of the function's behavior, purpose, and any side effects.
//  *
//  * @param[in]  input_name  Description of the input parameter.
//  * @param[out] output_name Description of the output parameter (if any).
//  * @return ReturnType Description of the return value.
//  *
//  * @note Optional notes about limitations, assumptions, etc.
//  * @warning Optional warning about misuse or important constraints.
//  * @see Optional references to related functions/classes/files.
//  */