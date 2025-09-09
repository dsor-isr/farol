#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

class TrajectoryPlanner {
public:
  TrajectoryPlanner();

  // Same API
  bool plan(double x0, double y0, double z0, double psi0,
            double dx_goal,
            double u_term,
            double vmax_u, double vmax_v,
            double amax_t,
            double wmax, double awmax,
            double rmax, double armax);

  void evaluate(double t,
                double& x, double& y, double& z,
                double& xd, double& yd, double& zd,
                double& xdd, double& ydd, double& zdd,
                double& yaw, double& r, double& ar) const;

  double getTotalTime() const { return T_total_; }
  bool   isPlanned()    const { return planned_; }

private:
  // -------- utilities --------
  struct Quintic { double c0=0,c1=0,c2=0,c3=0,c4=0,c5=0, T=0; };
  Quintic makeQuintic(double s0, double v0, double a0,
                      double sT, double vT, double aT, double T) const;
  void    evalQuintic(const Quintic& q, double t,
                      double& s, double& v, double& a) const;

  struct TrapTime { double T=0, T_acc=0, T_cruise=0, T_dec=0, v_peak=0; bool triangular=false; };
  TrapTime trapezoid_time(double L, double v0, double vT,
                          double vmax, double amax) const;

  inline double clamp01(double t, double T) const {
    if (t < 0.0) return 0.0; if (t > T) return T; return t;
  }
  inline double wrapToPi(double a) const {
    double r = std::fmod(a + M_PI, 2.0*M_PI);
    if (r < 0) r += 2.0*M_PI;
    return r - M_PI;
  }
  inline double angleDiff(double to, double from) const { return wrapToPi(to - from); }

  // -------- spatial sampling --------
  struct Sample {
    double s=0;     // arc length along full path
    double x=0, y=0;
    double psi=0;
    double kappa=0; // curvature
  };

  // Straight sampling from (x0,y0,psi) over length L at step ds
  void buildStraight(double x0, double y0, double psi, double L,
                     std::vector<Sample>& path, double ds) const;

  // Increment integrators (no dependencies)
  void integrateClothoid(double& x, double& y, double& psi,
                         double k0, double sigma, double L, double ds,
                         std::vector<Sample>* path = nullptr) const;

  void integrateArc(double& x, double& y, double& psi,
                    double kappa, double L, double ds,
                    std::vector<Sample>* path = nullptr) const;

  // Build a CAC block (0->kR, arc kR, kR->0) at world pose start (x0,y0,psi0)
  void buildCAC(double x0, double y0, double psi0,
                double kR, double sigma, double Lc, double La,
                std::vector<Sample>& path, double ds) const;

  // Compute CAC net displacement starting at origin/heading=0 (no sampling)
  struct CACResult { double dx=0, dy=0, dpsi=0, Lc=0, La=0; };
  CACResult cac_displacement(double kR, double sigma, double dpsi_total, double ds) const;

  // Time parameterization with constant surge u_term
  void parameterizeTimeConstantSpeed(const std::vector<Sample>& path,
                                     double u_term,
                                     std::vector<double>& t_of_s) const;

  // Depth profile
  void planDepth(double z0, double wmax, double awmax);
  void evalDepth(double t, double& z, double& zd, double& zdd) const;

  // --------- cached plan ----------
  bool planned_{false};

  // Start + goal
  double x0_=0, y0_=0, z0_=0, psi0_=0;
  double x_goal_=0, y_goal_=0;

  // Params
  double u_term_=0;
  double vmax_u_=1.0, vmax_v_=0.5;
  double amax_t_=0.3;
  double wmax_=0.5, awmax_=0.7;
  double rmax_=M_PI/6.0, armax_=M_PI/3.0;

  // Derived clothoid parameters
  double kR_=0.0;     // 1/R_min = rmax/u_term
  double sigma_=0.0;  // dκ/ds = armax / u_term^2
  double Lc_=0.0;     // kR_/sigma_
  double La_=0.0;

  // Phases (lengths)
  double L_LOS_=0.0;
  double L_CAC_=0.0;
  double L_final_=0.0;

  // Path + time
  std::vector<Sample> path_;
  std::vector<double> t_of_s_;
  double T_total_=0.0;

  // Depth
  Quintic depth_q_;
  double Tz_=0.0;
};
