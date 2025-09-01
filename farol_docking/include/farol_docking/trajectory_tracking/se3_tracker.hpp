#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <farol_docking/trajectory_tracking/controller_base.hpp>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


class Se3Tracker : public ControllerBase {
  public:
    Se3Tracker(ros::NodeHandle* nh, ros::NodeHandle* pnh);
    void compute_wrench(double dt) override;


    // Gains (diagonal)
    Eigen::Vector3d Kpv_, Kdv_, Kiv_;
    Eigen::Vector3d KpR_, KdW_, KiR_;


    // Integrators
    Eigen::Vector3d z_p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d z_R_{Eigen::Vector3d::Zero()};


    // Derivative filters (Hz)
    Eigen::Vector3d fc_v_{Eigen::Vector3d::Constant(5.0)};
    Eigen::Vector3d fc_w_{Eigen::Vector3d::Constant(5.0)};
    Eigen::Vector3d e_v_filt_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d e_w_filt_{Eigen::Vector3d::Zero()};


    // Anti‑windup back‑calculation gains
    Eigen::Vector3d kaw_v_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d kaw_w_{Eigen::Vector3d::Zero()};


    // Simple model
    Eigen::Vector3d M_{49.3, 50.0, 99.3}; // surge, sway, heave
    Eigen::Vector3d Jdiag_{1.0, 1.0, 3.0};
    Eigen::Vector3d Dlin_{20.0,20.0,20.0};
    Eigen::Vector3d Dang_{2.0, 2.0, 4.0};


    // Selection (4‑DoF)
    Eigen::Matrix<double,6,6> Ssel_;


    // Optional wrench limits
    Eigen::Vector3d Fmin_{-1e9,-1e9,-1e9}, Fmax_{1e9,1e9,1e9};
    Eigen::Vector3d Mmin_{-1e9,-1e9,-1e9}, Mmax_{1e9,1e9,1e9};


    private:
    inline double alpha(double fc, double dt) const { return std::exp(-2.0*M_PI*fc*dt); }
};

// se3_tracker.hpp
// #pragma once
// #include "controller_base.hpp"

// class Se3Tracker : public ControllerBase {
// public:
//   Se3Tracker(ros::NodeHandle* nh, ros::NodeHandle* pnh);

//   // Main step: compute wrench tau = [F;M]
//   void compute_wrench(double dt) override; 

//   // Gains (diagonal)
//   Eigen::Vector3d Kpv_, Kdv_, Kiv_;
//   Eigen::Vector3d KpR_, KdW_, KiR_;

//   // Integrators
//   Eigen::Vector3d z_p_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d z_R_{Eigen::Vector3d::Zero()};

//   // Derivative filter (on error rates) per axis (Hz)
//   Eigen::Vector3d fc_v_{Eigen::Vector3d::Constant(5.0)};
//   Eigen::Vector3d fc_w_{Eigen::Vector3d::Constant(5.0)};
//   Eigen::Vector3d e_v_filt_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d e_w_filt_{Eigen::Vector3d::Zero()};

//   // Anti-windup back-calculation
//   Eigen::Vector3d kaw_v_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d kaw_w_{Eigen::Vector3d::Zero()};

//   // Model terms (start simple; you can replace by your real M,C,D,g later)
//   double m_{25.0};                         // kg
//   Eigen::Vector3d Jdiag_{1.0, 1.0, 3.0};   // kg m^2
//   Eigen::Vector3d Dlin_{20.0,20.0,20.0};   // linear damping
//   Eigen::Vector3d Dang_{2.0, 2.0, 4.0};    // angular damping

//   // Selection (4-DoF actuation)
//   Eigen::Matrix<double,6,6> Ssel_;

//   // Optional saturations on wrench (pre-allocator)
//   Eigen::Vector3d Fmin_{-1e9,-1e9,-1e9}, Fmax_{1e9,1e9,1e9};
//   Eigen::Vector3d Mmin_{-1e9,-1e9,-1e9}, Mmax_{1e9,1e9,1e9};

// private:
//   inline double alpha(double fc, double dt) const { return std::exp(-2.0*M_PI*fc*dt); }
// };
