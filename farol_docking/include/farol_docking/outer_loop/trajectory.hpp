#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <cmath>
#include <algorithm>

class TrajectoryPlanner {
public:
  TrajectoryPlanner();

  // API preserved
  bool plan(double x0, double y0, double z0, double psi0,
            double dx_goal,
            double u_term,
            double vmax_u, double vmax_v,
            double amax_t,
            double wmax, double awmax,
            double rmax, double armax,
            double jerk_ratio);

  void evaluate(double t,
                double& x, double& y, double& z,
                double& xd, double& yd, double& zd,
                double& xdd, double& ydd, double& zdd,
                double& yaw, double& r, double& ar) const;

  double getTotalTime() const { return timeline_.T_total; }

private:
  // -------- math helpers --------
  static inline double clamp01(double t, double T){
    if (t < 0.0) 
        return 0.0; 
    if (t > T) 
        return T; 
    return t;
  }
  static constexpr double PI = 3.14159265358979323846;
  static inline double wrapToPi(double a){
    double r = std::fmod(a + PI, 2.0*PI); if (r < 0) r += 2.0*PI; return r - PI;
  }
  static inline double angleDiff(double to, double from){ return wrapToPi(to - from); }

  // -------- 16-pt Gauss–Legendre --------
  static const double GL16_x[8];
  static const double GL16_w[8];

  // -------- jerk-limited S-curve law --------
  struct SCurve {
    // inputs
    double s0=0.0, v0=0.0, vT=0.0;
    double vmax=0.0, amax=0.0, jmax=0.0;
    // solved
    double v_peak=0.0;
    double ta_j=0.0, ta_a=0.0, ta_dn=0.0;
    double t_cruise=0.0;
    double td_j=0.0, td_a=0.0, td_dn=0.0;
    double T=0.0;
  };
  struct HalfRes { double s=0, t_j=0, t_a=0, t_dn=0; };

  static HalfRes half_jerk_profile(double dv, double amax, double jmax);
  static SCurve makeSCurve_L(double L, double v0, double vT,
                             double vmax, double amax, double jmax);
  static void evalSCurve(const SCurve& P, double t,
                         double& s, double& v, double& a);
  // shim so existing code-style "evalQuintic" calls still work
  static inline void evalQuintic(const SCurve& P, double t,
                                 double& s,double& v,double& a){
    evalSCurve(P,t,s,v,a);
  }

private:
  // planned state
  bool planned_{false};
  double x0_=0,y0_=0,z0_=0,psi0_=0;
  double dx_goal_=0, u_term_=0;

  // geometry
  double psi_LOS_=0;
  Eigen::Vector2d d_hat_{1.0,0.0};
  double L_total_=0;

  // pre-blend boundary
  Eigen::Vector2d P_pre_{0.0,0.0};
  Eigen::Vector2d V_pre_{0.0,0.0};
  Eigen::Vector2d A_pre_{0.0,0.0};
  double L2_=0;

  // blend footprint + final
  double IxF_{0.0}, IyF_{0.0};
  double x_end_blend_{0.0};
  double tail_dir_{0.0};

  // limits
  double vmax_u_=0, vmax_v_=0, amax_t_=0;
  double wmax_=0, awmax_=0;        // depth vel/acc caps
  double rmax_=0, armax_=0;        // yaw rate/acc caps
  double jtrans_=0, jdepth_=0, jyaw_=0; // jerk caps derived

  // timeline
  struct Timeline {
    double T1=0, T2=0, T3=0, T4=0, Tz=0;
    double t0_yaw_in=0, t0_los=0, t0_blend=0, t0_final=0;
    double T_total=0;
  } timeline_;

  // motion laws
  SCurve yaw_in_q_, los_q_, yaw_out_q_, depth_q_;
};
