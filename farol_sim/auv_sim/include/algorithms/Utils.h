/**
 * @brief     Utils.h - Implements basic mathematical functions that can be used anywhere
 * @author    Marcelo Jacinto
 * @version   1.0.0
 * @date      2021/11/12
 * @copyright MIT
 */
#pragma once
#include <cmath>

/**
 * Compute the integral of a state (expressed as an Eigen::Vector) using Euler's method
 * @param dt
 * @param state
 * @param state_dot
 * @return
 */
inline Eigen::VectorXd eulerIntegration(double dt, const Eigen::VectorXd &state, const Eigen::VectorXd &state_dot) {
    return state + (dt * state_dot);
}

/**
 * Function that wraps an angle between 0 and 2PI
 * @param angle An angle expressed in radians
 * @return A wrapped angle between 0 and 2PI (in radians)
 */
inline double wrapAngle(double angle) {

    double wrapped_angle = std::fmod(angle, 2 * M_PI);
    if(wrapped_angle < 0) wrapped_angle += 2 * M_PI;
    return wrapped_angle;
}

/**
 *
 * @param phi Roll angle - rotation about the x-axis (rad)
 * @param theta Pitch angle - rotation about the y-axis (rad)
 * @param psi Yaw angle - rotation about the z-axis (rad)
 * @return
 */
inline Eigen::Matrix3d rotationBodyToInertial(double phi, double theta, double psi) {

    Eigen::Matrix3d rot_matrix;
    rot_matrix << cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * cos(phi) * sin(theta)),
                  sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(phi) * sin(theta) * sin(psi)), (-cos(psi) * sin(phi)) + (sin(theta) * sin(psi) * cos(phi)),
                 -sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi);
    return rot_matrix;
}

/**
 *
 * @param phi
 * @param theta
 * @param psi
 * @return
 */
inline Eigen::Matrix3d rotationAngularBodyToInertial(double phi, double theta, double psi) {
    Eigen::Matrix3d transformation_matrix;
    transformation_matrix << 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
                             0, cos(phi), -sin(phi),
                             0, sin(phi) / cos(theta), cos(phi) / cos(theta);
    return transformation_matrix;
}

/**
 *
 * @param vector
 * @return
 */
inline Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d &vector) {

    Eigen::Matrix3d skew_symmetric;
    skew_symmetric <<       0.0, -vector(2),  vector(1),
                      vector(2),        0.0, -vector(0),
                     -vector(1),  vector(0),        0.0;

    return skew_symmetric;
}

/**
 *
 * @param value
 * @param min
 * @param max
 * @return
 */
inline double saturation(double value, double min, double max) {
    return std::max(std::min(value, max), min);
}