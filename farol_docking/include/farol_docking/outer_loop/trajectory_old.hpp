#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

class TrajectoryPlanner {
public:
  // ====================
  // Constructor
  // ====================
  TrajectoryPlanner();

  // ====================
  // Public API
  // ====================

  /**
   * @brief Plan a full 4-phase trajectory given initial pose and target
   *
   * @param x0, y0, z0   Initial position
   * @param psi0          Initial yaw (rad)
   * @param dx_goal       Final x target (goal is [dx_goal, 0, 0])
   * @param u_term        Desired terminal surge speed
   * @param vmax_u, vmax_v Velocity limits for surge/sway
   * @param amax_t        Max planar acceleration
   * @param wmax, awmax   Depth velocity and acceleration limits
   * @param rmax, armax   Yaw rate and yaw acceleration limits
   * @return true if planning succeeded
   */
  bool plan(double x0, double y0, double z0, double psi0,
            double dx_goal,
            double u_term,
            double vmax_u, double vmax_v,
            double amax_t,
            double wmax, double awmax,
            double rmax, double armax);

  /**
   * @brief Evaluate trajectory at time t since planning.
   */
  void evaluate(double t,
                double& x, double& y, double& z,
                double& xd, double& yd, double& zd,
                double& xdd, double& ydd, double& zdd,
                double& yaw, double& r, double& ar) const;

  /// Total duration of the trajectory
  double getTotalTime() const { return timeline_.T_total; }

  /// Whether plan() was successfully called
  bool isPlanned() const { return planned_; }

private:
  // ====================
  // Internal structures
  // ====================
  struct Quintic {
    double c0=0, c1=0, c2=0, c3=0, c4=0, c5=0;
    double T=0;
  };

  struct TrapTime {
    double T{0}, T_acc{0}, T_cruise{0}, T_dec{0}, v_peak{0};
    bool triangular{false};
  };

  struct Timeline {
    double T1=0, T2=0, T3=0, T4=0, Tz=0;
    double t0_yaw_in=0, t0_los=0, t0_blend=0, t0_final=0;
    double T_total=0;
  };

  // ====================
  // Internal helpers
  // ====================
  Quintic makeQuintic(double s0, double v0, double a0,
                      double sT, double vT, double aT, double T) const;
  void evalQuintic(const Quintic& q, double t, double& s, double& v, double& a) const;

  TrapTime trapezoid_time(double L, double v0, double vT,
                          double vmax, double amax) const;

  // ====================
  // Internal state
  // ====================
  bool planned_{false};

  // Cached start state
  double x0_=0, y0_=0, z0_=0, psi0_=0;
  double dx_goal_=0, u_term_=0;

  // LOS
  double psi_LOS_=0;
  Eigen::Vector2d d_hat_{1.0,0.0};
  double L_total_=0;

  // Pre-blend
  Eigen::Vector2d P_pre_{0.0,0.0};
  Eigen::Vector2d V_pre_{0.0,0.0};
  Eigen::Vector2d A_pre_{0.0,0.0};
  double L2_=0;

  Timeline timeline_;

  // Quintics
  Quintic yaw_in_q_, los_q_, blend_x_q_, blend_y_q_, yaw_out_q_, depth_q_;
};
