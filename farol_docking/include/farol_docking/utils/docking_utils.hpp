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
#include <optional>
#include <array>
#include <random>
#include <algorithm>

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

  
inline geometry_msgs::Vector3 toMsg(const Eigen::Vector3d& v) {
  geometry_msgs::Vector3 msg;
  msg.x = v.x(); msg.y = v.y(); msg.z = v.z();
  return msg;
}

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

inline Eigen::Vector3d be_to_xyz(double bearing, double elevation) {
    Eigen::Vector3d xyz; 
    xyz(0) = cos(elevation) * cos(bearing);
    xyz(1) = cos(elevation) * sin(bearing);
    xyz(2) = sin(elevation);
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

// gets a rotation matrix from B to I and outputs the angles measured from I to B
// which is the standard for representing eulçer angles. The rotation matric from 
// B to I is the standard when it comes to rotation matrices.
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



inline Eigen::Matrix3d rpyToRot(double roll, double pitch, double yaw)
{
    const Eigen::AngleAxisd Rx(roll,  Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd Rz(yaw,   Eigen::Vector3d::UnitZ());
    // ZYX order: yaw -> pitch -> roll
    return (Rz * Ry * Rx).toRotationMatrix();
}




inline std::optional<double> yaw_from_two_usbl_rbe(const Eigen::Vector3d& rbe_B, const Eigen::Vector3d& rbe_D)
{
  const Eigen::Vector2d pB = rbe_to_xyz(rbe_B).head<2>();
  const Eigen::Vector2d pD = rbe_to_xyz(rbe_D).head<2>();
  const double nB = pB.norm(), nD = pD.norm();
  if (nB < 1e-9 || nD < 1e-9) return std::nullopt; // undefined yaw

  const Eigen::Vector2d uB = (pB / nB);
  const Eigen::Vector2d uD = (pD / nD);

  const Eigen::Vector2d qB = -uB; // rays oppose each other
  const double r1 = uD.dot(qB);
  const double r2 = uD.x()*qB.y() - uD.y()*qB.x(); // 2D cross z
  return -std::atan2(r2, r1); // radians in (-pi, pi]
}



static inline bool inject_outliers_rbe(Eigen::Vector3d* z, double p_outlier = 0.07)
{
    // ---- constants (no <numbers> to keep C++17) ----
    constexpr double PI   = 3.14159265358979323846;
    constexpr double PI_2 = PI / 2.0;

    // ---- tunables (tweak to taste) ----
    const double wall_az   = 0.0;   // wall/dock face normal (radians)
    const double p_wall    = 0.55;  // fraction of outliers that are wall reflections
    const double p_surface = 0.40;  // fraction that are surface/bottom reflections
    const double min_bias  = 0.5;   // +range bias [m] (reflections)
    const double max_bias  = 5.0;   // +range bias [m]
    const double spike_r   = 2.0;  // +range for wild spikes

    auto wrapPi = [&](double a) {
        a = std::fmod(a + PI, 2.0 * PI);
        if (a < 0) a += 2.0 * PI;
        return a - PI;             // [-pi, pi]
    };
    auto sec_like = [&](double x) { return 1.0 / std::max(0.15, std::cos(x)); }; // avoid blow-up

    // RNG (thread-local)
    thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<double> U01(0.0, 1.0);
    auto U = [&](double a, double b) { return a + (b - a) * U01(rng); };

    if (!z) return true; // per your requirement: always return true

    double& r = (*z)(0);
    double& b = (*z)(1);
    double& e = (*z)(2);

    // Maybe inject an outlier; otherwise leave vector unchanged
    if (U01(rng) >= p_outlier) {
        return true; // no change, as requested
    }

    const double choice = U01(rng);
    if (choice < p_wall) {
        // --- Vertical wall reflection: mirror bearing, +range bias ---
        const double b_img = wrapPi(2.0 * wall_az - b);

        double inc = std::fabs(wrapPi(b - wall_az));     // [0, pi)
        if (inc > PI_2) inc = PI - inc;                  // fold to [0, pi/2]
        const double bias = U(min_bias, max_bias) * sec_like(inc);

        r = std::max(0.0, r + bias);
        b = b_img;                                       // elevation ~ unchanged
        // e can remain as-is; keeping it simple/realistic

    } else if (choice < p_wall + p_surface) {
        // --- Surface/bottom reflection: flip elevation, +range bias ---
        const double e_img = wrapPi(-e);

        double inc = std::min(std::fabs(e), PI_2);       // grazing to horizontal plane
        const double bias = U(min_bias, max_bias) * sec_like(inc);

        r = std::max(0.0, r + bias);
        e = e_img;                                       // bearing ~ unchanged

    } else {
        // --- Rare wild spike: random AoA + big +range (sidelobe/peak swap) ---
        b = wrapPi(U(-PI, PI));
        e = wrapPi(U(-PI_2, PI_2));
        r = std::max(0.0, r + spike_r * (0.5 + U01(rng))); // +[6..12] m
    }

    return true; // per your request
}



// dvl_noise.hpp


// Add noise + distance-dependent outliers to a DVL velocity (m/s).
// - Modifies vel in place.
// - distance_m is distance to the dock (meters).
// - Returns true if an outlier was applied.
inline bool corruptDvlMeasurement(Eigen::Vector3d& vel,
                                  double distance_m,
                                  double noise_sigma = 0.01,      // m/s (per-axis)
                                  double outlier_min_scale = 2.0, // × speed
                                  double outlier_max_scale = 8.0)
    {
        // RNGs (thread-safe and cheap to reuse)
        static thread_local std::mt19937 rng{std::random_device{}()};
        std::normal_distribution<double> N(0.0, noise_sigma);
        std::uniform_real_distribution<double> U01(0.0, 1.0);
        std::uniform_real_distribution<double> U02(0.0, 1.0);
        std::uniform_real_distribution<double> Uscale(outlier_min_scale, outlier_max_scale);

        // 1) Small Gaussian noise (per component)
        vel.x() += N(rng);
        vel.y() += N(rng);
        vel.z() += N(rng);

        // 2) Outlier probability: 0.5 when d<=2 m, 0.05 when d>=10 m, linear in-between
        double p;
        if (distance_m <= 2.0) {
            p = 0.2;
        } else if (distance_m >= 10.0) {
            p = 0.02;
        } else {
            double t = (distance_m - 2.0) / 8.0;       // 0..1 across [2,10]
            p = 0.2 + (0.02 - 0.2) * t;                // linear interp: 0.5 -> 0.05
        }
        


        // 3) With probability p, spike the velocity magnitude by 4–10×
        bool made_outlier = (U01(rng) < p);
        int sign;
        if (made_outlier) {
            sign = (U02(rng) < 0.5) ? -1 : 1;
            double s = Uscale(rng) * sign;
            vel *= s;                                  // scale entire 3D velocity
        }
        return made_outlier;
    }
