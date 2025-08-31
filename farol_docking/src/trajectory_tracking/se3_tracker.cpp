// se3_tracker.cpp
#include "se3_tracker.hpp"
#include <algorithm>

Se3Tracker::Se3Tracker(ros::NodeHandle* nh, ros::NodeHandle* pnh)
: ControllerBase(nh,pnh)
{
  // === Load params (diagonal gains) ===
  auto getv = [&](const std::string& key, const Eigen::Vector3d& def){
    std::vector<double> v; nh_private_.param<std::vector<double>>(key, v, {def.x(),def.y(),def.z()});
    return Eigen::Vector3d(v[0], v[1], v[2]);
  };
  Kpv_ = getv("se3/Kpv", Eigen::Vector3d::Constant(50.0));
  Kdv_ = getv("se3/Kdv", Eigen::Vector3d::Constant(40.0));
  Kiv_ = getv("se3/Kiv", Eigen::Vector3d::Constant(0.5));
  KpR_ = getv("se3/KpR", Eigen::Vector3d(0,0,3.0));   // only yaw active
  KdW_ = getv("se3/KdW", Eigen::Vector3d(0,0,2.0));
  KiR_ = getv("se3/KiR", Eigen::Vector3d(0,0,0.3));

  fc_v_ = getv("se3/fc_v", Eigen::Vector3d::Constant(5.0));
  fc_w_ = getv("se3/fc_w", Eigen::Vector3d::Constant(5.0));
  kaw_v_= getv("se3/kaw_v", Eigen::Vector3d::Zero());
  kaw_w_= getv("se3/kaw_w", Eigen::Vector3d::Zero());

  nh_private_.param("se3/mass", m_, 25.0);
  std::vector<double> Jv;
  nh_private_.param<std::vector<double>>("se3/Jdiag", Jv, {1.0,1.0,3.0});
  Jdiag_ = Eigen::Vector3d(Jv[0],Jv[1],Jv[2]);

  // Damping
  Dlin_ = getv("se3/Dlin", Eigen::Vector3d(20,20,20));
  Dang_ = getv("se3/Dang", Eigen::Vector3d(2,2,4));

  // Wrench limits (optional)
  Fmin_ = getv("se3/Fmin", Eigen::Vector3d::Constant(-1e9));
  Fmax_ = getv("se3/Fmax", Eigen::Vector3d::Constant( 1e9));
  Mmin_ = getv("se3/Mmin", Eigen::Vector3d::Constant(-1e9));
  Mmax_ = getv("se3/Mmax", Eigen::Vector3d::Constant( 1e9));

  // Selection: 4-DoF (zero roll,pitch moments)
  Ssel_.setZero(); Ssel_.diagonal() << 1,1,1,0,0,1;
}

void Se3Tracker::compute_wrench(double dt)
{
  // --- Build desired body signals ---
  // v_d = R_d^T p_dot_d
  Eigen::Vector3d v_d = R_d_.transpose() * pd_d_;
  // vdot_d (compatible with kinematics)
  Eigen::Vector3d vdot_d = R_d_.transpose()*pdd_d_ - hat(w_d_) * v_d;

  // --- Errors (right-invariant) ---
  Eigen::Matrix3d Rt = R_.transpose();
  Eigen::Vector3d e_b = Rt * (position_ - p_d_);
  Eigen::Vector3d e_v = v_ - Rt*R_d_*v_d;
  Eigen::Vector3d e_R = 0.5 * vee(R_d_.transpose()*R_ - R_.transpose()*R_d_);
  Eigen::Vector3d e_w = w_ - Rt*R_d_*w_d_;

  // --- Integral states ---
  z_p_ += e_b * dt;
  z_R_ += e_R * dt;

  // --- Filter derivative-like terms (here we filter e_v, e_w) ---
  for(int i=0;i<3;++i){
    double av = alpha(fc_v_(i), dt);
    e_v_filt_(i) = av*e_v_filt_(i) + (1.0-av)*e_v(i);
    double aw = alpha(fc_w_(i), dt);
    e_w_filt_(i) = aw*e_w_filt_(i) + (1.0-aw)*e_w(i);
  }

  // --- Virtual accelerations ---
  Eigen::Vector3d vdot_star =
      vdot_d - hat(w_) * Rt*R_d_*v_d
      - Kdv_.cwiseProduct(e_v_filt_)
      - Kpv_.cwiseProduct(e_b)
      - Kiv_.cwiseProduct(z_p_);

  Eigen::Vector3d wdot_star =
      Rt*R_d_*wdd_d_ - hat(w_) * Rt*R_d_*w_d_
      - KdW_.cwiseProduct(e_w_filt_)
      - KpR_.cwiseProduct(e_R)
      - KiR_.cwiseProduct(z_R_);

  // --- Inverse dynamics (simple diagonal M,D; add g(.) if you have) ---
  Eigen::Matrix3d J = Jdiag_.asDiagonal();
  Eigen::Vector3d F = m_ * vdot_star + Dlin_.cwiseProduct(v_);
  Eigen::Vector3d M = J * wdot_star + Dang_.cwiseProduct(w_);

  // Apply selection
  Eigen::Matrix<double,6,1> tau;
  tau << F, M;
  tau = Ssel_ * tau;

  // Optional pre-allocator saturation for anti-windup
  Eigen::Vector3d Fsat = tau.segment<3>(0).cwiseMax(Fmin_).cwiseMin(Fmax_);
  Eigen::Vector3d Msat = tau.segment<3>(3).cwiseMax(Mmin_).cwiseMin(Mmax_);
  Eigen::Matrix<double,6,1> tausat; tausat << Fsat, Msat;

  // Anti-windup (back-calculation on z_p_, z_R_) using saturated wrench
  // NOTE: If your allocator returns the actually applied tau_applied, use that instead of tausat.
  Eigen::Matrix<double,6,1> err_tau = tausat - tau;
  // Split into translational/rotational parts component-wise
  z_p_ += (kaw_v_.cwiseProduct(err_tau.segment<3>(0)).cwiseQuotient((Kiv_.array()+1e-9).matrix())) * dt;
  z_R_ += (kaw_w_.cwiseProduct(err_tau.segment<3>(3)).cwiseQuotient((KiR_.array()+1e-9).matrix())) * dt;

  // Output
  force_  = Fsat;           // what we intend to send
  torque_ = Msat;
}
