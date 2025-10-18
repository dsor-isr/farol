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
	Kpp_ = getv("se3/Kpp", Eigen::Vector3d::Constant(0.0));
	Kdv_ = getv("se3/Kdv", Eigen::Vector3d::Constant(0.0));
	Kip_ = getv("se3/Kip", Eigen::Vector3d::Constant(0.0));
	KpR_ = getv("se3/KpR", Eigen::Vector3d(0,0,0.0));
	Kdw_ = getv("se3/Kdw", Eigen::Vector3d(0,0,0.0));
	KiR_ = getv("se3/KiR", Eigen::Vector3d(0,0,0.0));


	fc_v_= getv("se3/fc_v", Eigen::Vector3d::Constant(10.0));
	fc_w_= getv("se3/fc_w", Eigen::Vector3d::Constant(10.0));
	kaw_v_=getv("se3/kaw_v", Eigen::Vector3d::Zero());
	kaw_w_=getv("se3/kaw_w", Eigen::Vector3d::Zero());


	M_ = getv("se3/M_diag", Eigen::Vector3d::Constant(1.0));

	std::vector<double> Jv; nh_private_.param<std::vector<double>>("se3/Jdiag", Jv, {1.0,1.0,1.0});
	Jdiag_ = Eigen::Vector3d(Jv[0],Jv[1],Jv[2]);
	Dlin_ = getv("se3/Dlin", Eigen::Vector3d(0,0,0));
	Dang_ = getv("se3/Dang", Eigen::Vector3d(0,0,0));


	Fmin_ = getv("se3/Fmin", Eigen::Vector3d::Constant(-1e9));
	Fmax_ = getv("se3/Fmax", Eigen::Vector3d::Constant( 1e9));
	Mmin_ = getv("se3/Mmin", Eigen::Vector3d::Constant(-1e9));
	Mmax_ = getv("se3/Mmax", Eigen::Vector3d::Constant( 1e9));


	// 4‑DoF selection: zero Mx, My
	Ssel_.setZero();
	// Ssel_.diagonal() << 1,1,1,0,0,1;
  Ssel_.diagonal() << 1,1,1,1,1,1;

  set_gain_srv_ = nh_private_.advertiseService("/myellow0/docking/trajectory_tracking/set_gain", &Se3Tracker::setGainSrv, this);
  // set_gain_srv_ = nh_private_.advertiseService("/bluerov_heavy0/docking/trajectory_tracking/set_gain", &Se3Tracker::setGainSrv, this);

  ROS_INFO_STREAM("Trajectory Tracking Controller Parameters:\n--- Gains ---\n"<<
                  "Kpp: "<< Kpp_ <<
                  "\nKdv: "<< Kdv_ <<
                  "\nKip: "<< Kip_ <<
                  "\nKpR: "<< KpR_ <<
                  "\nKdw: "<< Kdw_ <<
                  "\nKiR: "<< KiR_ <<
                  "\nKpp: "<< Kpp_ <<
                  "\n--- Model ---\n" <<
                  "\nM: " << M_ <<
                  "\nJ: " << Jdiag_ <<
                  "\nDlin: " << Dlin_<<
                  "\nDang: " << Dang_<<
                  "\n--- Saturations ---\n" <<
                  "\nFmin: "<< Fmin_ <<
                  "\nFmax: "<< Fmax_ <<
                  "\nMmin: "<< Mmin_ <<
                  "\nMmax: "<< Mmax_ <<
                  "\n--- Velocity low pass ---\n" <<
                  "\nfc_v: " << fc_v_ <<
                  "\nfc_w: " << fc_w_ <<
                  "\n--- Anti-windup ---\n" <<
                  "\nkaw_v: " << kaw_v_ <<
                  "\nkaw_w: " << kaw_w_ );
}

void Se3Tracker::compute_wrench(double dt)
{
  if (dt <= 0.0) return; 

  // Desired body signals
  const Eigen::Vector3d v_d    = R_d_.transpose() * pd_d_;
  // (vdot_d is in desired body frame; don't mix it directly later)
  // const Eigen::Vector3d vdot_d = R_d_.transpose() * pdd_d_ - hat(w_d_) * v_d;

  // Errors (right-invariant)
  const Eigen::Matrix3d Rt = R_.transpose();
	
  const Eigen::Vector3d e_b = Rt * (position_ - p_d_);
  const Eigen::Vector3d e_v = v_  - Rt*R_d_*v_d;
  const Eigen::Vector3d e_R = 0.5 * vee(R_d_.transpose()*R_ - R_.transpose()*R_d_);
  const Eigen::Vector3d e_w = w_  - Rt*R_d_*w_d_;


  // Integrators 
  z_p_ += e_b * dt;
  z_R_ += e_R * dt;

  // ---- Filter e_v and e_w (init to first sample) ----
  e_v_filt_ = e_v; e_w_filt_ = e_w;
  // static bool first = true;
  // if (first) { e_v_filt_ = e_v; e_w_filt_ = e_w;} first = false; }
  // for (int i=0; i<3; ++i) {
  //   const double av = alpha(fc_v_(i), dt);
  //   e_v_filt_(i) = av*e_v_filt_(i) + (1.0-av)*e_v(i);
  //   const double aw = alpha(fc_w_(i), dt);
  //   e_w_filt_(i) = aw*e_w_filt_(i) + (1.0-aw)*e_w(i);
  // }

  //Eigen::Vector3d vdot_d = R_d_.transpose()*pdd_d_ - hat(w_d_) * v_d;
  //Eigen::Vector3d vdot_star = vdot_d - hat(w_) * Rt*R_d_*v_d   - Kdv_.cwiseProduct(e_v_filt_) - Kpp_.cwiseProduct(e_b) - Kip_.cwiseProduct(z_p_)
  
  // ---- Virtual accelerations ----
  const Eigen::Vector3d vdot_ff = Rt * pdd_d_ - hat(w_) * Rt*R_d_*v_d;
  const Eigen::Vector3d wdot_ff = Rt*R_d_*wdd_d_ - hat(w_) * Rt*R_d_*w_d_;

  const Eigen::Vector3d vdot_star =
      vdot_ff - Kdv_.cwiseProduct(e_v_filt_) - Kpp_.cwiseProduct(e_b) - Kip_.cwiseProduct(z_p_);

  const Eigen::Vector3d wdot_star =
      wdot_ff - Kdw_.cwiseProduct(e_w_filt_) - KpR_.cwiseProduct(e_R) - KiR_.cwiseProduct(z_R_);

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
  if (Kip_.maxCoeff() > 0.0 || KiR_.maxCoeff() > 0.0) {
    const Eigen::Matrix<double,6,1> err_tau = tausat - tau;
    if (Kip_.maxCoeff() > 0.0)
      z_p_ += (kaw_v_.cwiseProduct(err_tau.segment<3>(0)).cwiseQuotient((Kip_.array()+1e-9).matrix())) * dt;
    if (KiR_.maxCoeff() > 0.0)
      z_R_ += (kaw_w_.cwiseProduct(err_tau.segment<3>(3)).cwiseQuotient((KiR_.array()+1e-9).matrix())) * dt;
  }

  force_  = Fsat;
  torque_ = Msat;
}


bool Se3Tracker::setGainSrv(farol_docking::SetGain::Request& req,
                            farol_docking::SetGain::Response& res)
{
  auto key = req.name;                      // e.g. "Kpv", "Kpv.z", "se3/Kpv"
  auto vals = req.values;                   // len 1 or 3
  // Normalize: strip leading "se3/" if user passed full param path
  const std::string prefix = "se3/";
  if (key.compare(0, prefix.size(), prefix) == 0) key = key.substr(prefix.size());

  // Parse optional component suffix: ".x" ".y" ".z" or "[0]" "[1]" "[2]"
  int comp = -1;
  auto parse_component = [&](std::string& k) {
    if (!k.empty()) {
      // ".x/.y/.z"
      if (k.size() >= 2 && k[k.size()-2] == '.') {
        char c = k.back();
        if (c=='x') comp = 0; else if (c=='y') comp = 1; else if (c=='z') comp = 2;
        if (comp != -1) { k.erase(k.size()-2); return; }
      }
      // "[0]/[1]/[2]"
      if (k.size() >= 3 && k[k.size()-3] == '[' && k.back() == ']') {
        char c = k[k.size()-2];
        if (c=='0' || c=='1' || c=='2') { comp = c - '0'; k.erase(k.size()-3); return; }
      }
    }
  };
  parse_component(key);

  auto bad = [&](const std::string& msg){ res.ok=false; res.message=msg; return true; };

  auto set_vec3 = [&](Eigen::Vector3d& dst, const std::string& param_name) -> bool {
    if (comp >= 0) {
      if (vals.size() != 1) return bad("When targeting a single component use exactly one value");
      dst(comp) = vals[0];
    } else {
      if (vals.size() == 1) {
        dst.setConstant(vals[0]);
      } else if (vals.size() == 3) {
        dst = Eigen::Vector3d(vals[0], vals[1], vals[2]);
      } else {
        return bad("Provide either 1 value (broadcast) or 3 values (x,y,z)");
      }
    }
    // Reflect change to param server for persistence/visibility
    nh_private_.setParam(param_name, std::vector<double>{dst.x(), dst.y(), dst.z()});
    return true;
  };


  // Map known keys to variables + param names
  if      (key == "Kpp")  { if(!set_vec3(Kpp_,  "se3/Kpp"))  return true; }
  else if (key == "Kdv")  { if(!set_vec3(Kdv_,  "se3/Kdv"))  return true; }
  else if (key == "Kip")  { if(!set_vec3(Kip_,  "se3/Kip"))  return true; }

  else if (key == "KpR")  { if(!set_vec3(KpR_,  "se3/KpR"))  return true; }
  else if (key == "Kdw")  { if(!set_vec3(Kdw_,  "se3/Kdw"))  return true; }
  else if (key == "KiR")  { if(!set_vec3(KiR_,  "se3/KiR"))  return true; }

  else if (key == "fc_v") { if(!set_vec3(fc_v_, "se3/fc_v")) return true; }
  else if (key == "fc_w") { if(!set_vec3(fc_w_, "se3/fc_w")) return true; }

  else if (key == "kaw_v"){ if(!set_vec3(kaw_v_, "se3/kaw_v")) return true; }
  else if (key == "kaw_w"){ if(!set_vec3(kaw_w_, "se3/kaw_w")) return true; }

  else if (key == "M" || key=="M_diag" || key=="Mdiag") {
    if(!set_vec3(M_, "se3/M_diag")) return true;
  }
  else if (key == "J" || key=="Jdiag") {
    if(!set_vec3(Jdiag_, "se3/Jdiag")) return true;
  }
  else if (key == "Dlin") { if(!set_vec3(Dlin_, "se3/Dlin")) return true; }
  else if (key == "Dang") { if(!set_vec3(Dang_, "se3/Dang")) return true; }

  else if (key == "Fmin") { if(!set_vec3(Fmin_, "se3/Fmin")) return true; }
  else if (key == "Fmax") { if(!set_vec3(Fmax_, "se3/Fmax")) return true; }
  else if (key == "Mmin") { if(!set_vec3(Mmin_, "se3/Mmin")) return true; }
  else if (key == "Mmax") { if(!set_vec3(Mmax_, "se3/Mmax")) return true; }

  else {
    return bad("Unknown gain name: '" + req.name + "'");
  }

  res.ok = true;
  res.message = "Updated " + req.name;
  return true;
}
