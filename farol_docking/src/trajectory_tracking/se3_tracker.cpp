#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <farol_docking/trajectory_tracking/se3_tracker.hpp>



Se3Tracker::Se3Tracker(ros::NodeHandle* nh, ros::NodeHandle* pnh)
: ControllerBase(nh,pnh)
{
	auto getv = [&](const std::string& key, const Eigen::Vector3d& def){
		std::vector<double> v; nh_private_.param<std::vector<double>>(key, v, {def.x(),def.y(),def.z()});
		if(v.size()<3) v = {def.x(),def.y(),def.z()};
		return Eigen::Vector3d(v[0], v[1], v[2]);
	};
	Kpv_ = getv("se3/Kpv", Eigen::Vector3d::Constant(50.0));
	Kdv_ = getv("se3/Kdv", Eigen::Vector3d::Constant(40.0));
	Kiv_ = getv("se3/Kiv", Eigen::Vector3d::Constant(0.0));
	KpR_ = getv("se3/KpR", Eigen::Vector3d(0,0,3.0));
	KdW_ = getv("se3/KdW", Eigen::Vector3d(0,0,2.0));
	KiR_ = getv("se3/KiR", Eigen::Vector3d(0,0,0.0));


	fc_v_= getv("se3/fc_v", Eigen::Vector3d::Constant(5.0));
	fc_w_= getv("se3/fc_w", Eigen::Vector3d::Constant(5.0));
	kaw_v_=getv("se3/kaw_v", Eigen::Vector3d::Zero());
	kaw_w_=getv("se3/kaw_w", Eigen::Vector3d::Zero());


	M_ = getv("se3/M_diag", Eigen::Vector3d::Constant(50.0));

	std::vector<double> Jv; nh_private_.param<std::vector<double>>("se3/Jdiag", Jv, {1.0,1.0,3.0});
	Jdiag_ = Eigen::Vector3d(Jv[0],Jv[1],Jv[2]);
	Dlin_ = getv("se3/Dlin", Eigen::Vector3d(20,20,20));
	Dang_ = getv("se3/Dang", Eigen::Vector3d(2,2,4));


	Fmin_ = getv("se3/Fmin", Eigen::Vector3d::Constant(-1e9));
	Fmax_ = getv("se3/Fmax", Eigen::Vector3d::Constant( 1e9));
	Mmin_ = getv("se3/Mmin", Eigen::Vector3d::Constant(-1e9));
	Mmax_ = getv("se3/Mmax", Eigen::Vector3d::Constant( 1e9));


	// 4‑DoF selection: zero Mx, My
	Ssel_.setZero();
	Ssel_.diagonal() << 1,1,1,0,0,1;
}

void Se3Tracker::compute_wrench(double dt)
{
  if (dt <= 0.0) return;  // guard against zero/negative dt

  // Desired body signals (as before)
  const Eigen::Vector3d v_d    = R_d_.transpose() * pd_d_;
  // (vdot_d is in desired body frame; don't mix it directly later)
  // const Eigen::Vector3d vdot_d = R_d_.transpose() * pdd_d_ - hat(w_d_) * v_d;

  // Errors (right-invariant)
  const Eigen::Matrix3d Rt = R_.transpose();
	
  const Eigen::Vector3d e_b = Rt * (position_ - p_d_);
  const Eigen::Vector3d e_v = v_  - Rt*R_d_*v_d;
  const Eigen::Vector3d e_R = 0.5 * vee(R_d_.transpose()*R_ - R_.transpose()*R_d_);
  const Eigen::Vector3d e_w = w_  - Rt*R_d_*w_d_;


  // Integrators (won't contribute while Ki=0, but keep updated)
  z_p_ += e_b * dt;
  z_R_ += e_R * dt;

  // ---- Filter e_v and e_w (init to first sample) ----
  static bool first = true;
  if (first) { e_v_filt_ = e_v; e_w_filt_ = e_w;}// first = false; }
  // for (int i=0; i<3; ++i) {
  //   const double av = alpha(fc_v_(i), dt);
  //   e_v_filt_(i) = av*e_v_filt_(i) + (1.0-av)*e_v(i);
  //   const double aw = alpha(fc_w_(i), dt);
  //   e_w_filt_(i) = aw*e_w_filt_(i) + (1.0-aw)*e_w(i);
  // }

  //Eigen::Vector3d vdot_d = R_d_.transpose()*pdd_d_ - hat(w_d_) * v_d;
  //Eigen::Vector3d vdot_star = vdot_d - hat(w_) * Rt*R_d_*v_d   - Kdv_.cwiseProduct(e_v_filt_) - Kpv_.cwiseProduct(e_b) - Kiv_.cwiseProduct(z_p_)
  
  // ---- Virtual accelerations ----
  const Eigen::Vector3d vdot_ff = Rt * pdd_d_ - hat(w_) * Rt*R_d_*v_d;
  const Eigen::Vector3d wdot_ff = Rt*R_d_*wdd_d_ - hat(w_) * Rt*R_d_*w_d_;

  const Eigen::Vector3d vdot_star =
      vdot_ff - Kdv_.cwiseProduct(e_v_filt_) - Kpv_.cwiseProduct(e_b) - Kiv_.cwiseProduct(z_p_);

  const Eigen::Vector3d wdot_star =
      wdot_ff - KdW_.cwiseProduct(e_w_filt_) - KpR_.cwiseProduct(e_R) - KiR_.cwiseProduct(z_R_);

  // (Optional) clamp to accel limits if you added them as params
  // for (int i=0;i<3;++i) vdot_star(i) = amax_(i) * std::tanh(vdot_star(i)/std::max(1e-6, amax_(i)));
  // const double alphamax = std::max(1e-6, alphamax_z_); 
  // Eigen::Vector3d wdot_star_clamped(0,0, alphamax*std::tanh(wdot_star.z()/alphamax));


  // Inverse dynamics (diag M,D) -- if you have per-axis masses use m_vec_ here
  const Eigen::Matrix3d J = Jdiag_.asDiagonal();
  const Eigen::Vector3d F = M_.cwiseProduct(vdot_star) + Dlin_.cwiseProduct(v_);
  const Eigen::Vector3d M = J * wdot_star + Dang_.cwiseProduct(w_);

  Eigen::Matrix<double,6,1> tau; tau << F, M;
  tau = Ssel_ * tau;

  // Pre-allocator saturation (optional)
  const Eigen::Vector3d Fsat = tau.segment<3>(0).cwiseMax(Fmin_).cwiseMin(Fmax_);
  const Eigen::Vector3d Msat = tau.segment<3>(3).cwiseMax(Mmin_).cwiseMin(Mmax_);
  Eigen::Matrix<double,6,1> tausat; tausat << Fsat, Msat;

  // Anti-windup: if Ki==0, skip back-calculation to avoid 1e-9 division surprises
  if (Kiv_.maxCoeff() > 0.0 || KiR_.maxCoeff() > 0.0) {
    const Eigen::Matrix<double,6,1> err_tau = tausat - tau;
    if (Kiv_.maxCoeff() > 0.0)
      z_p_ += (kaw_v_.cwiseProduct(err_tau.segment<3>(0)).cwiseQuotient((Kiv_.array()+1e-9).matrix())) * dt;
    if (KiR_.maxCoeff() > 0.0)
      z_R_ += (kaw_w_.cwiseProduct(err_tau.segment<3>(3)).cwiseQuotient((KiR_.array()+1e-9).matrix())) * dt;
  }

  force_  = Fsat;
  torque_ = Msat;
}
