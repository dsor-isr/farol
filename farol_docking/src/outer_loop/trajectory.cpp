#include <farol_docking/outer_loop/trajectory.hpp>  

// 16-pt nodes/weights
const double TrajectoryPlanner::GL16_x[8] = {
  0.0950125098376374, 0.2816035507792589, 0.4580167776572274, 0.6178762444026438,
  0.7554044083550030, 0.8656312023878318, 0.9445750230732326, 0.9894009349916499
};
const double TrajectoryPlanner::GL16_w[8] = {
  0.1894506104550685, 0.1826034150449236, 0.1691565193950025, 0.1495959888165767,
  0.1246289712555339, 0.0951585116824928, 0.0622535239386479, 0.0271524594117541
};

TrajectoryPlanner::TrajectoryPlanner() = default;

// ---- S-curve helpers ----
static inline void seg_integ(double j, double dt, double &a, double &v, double &s) {
  // Exact primitives for constant jerk:
  // a(t) = a0 + j t
  // v(t) = v0 + a0 t + 0.5 j t^2
  // s(t) = s0 + v0 t + 0.5 a0 t^2 + (1/6) j t^3
  s += v*dt + 0.5*a*dt*dt + (1.0/6.0)*j*dt*dt*dt;
  v += a*dt + 0.5*j*dt*dt;
  a += j*dt;
}

TrajectoryPlanner::HalfRes
TrajectoryPlanner::half_jerk_profile(double dv, double amax, double jmax) {
  HalfRes H{};
  dv = std::max(0.0, dv);
  const double dv_tri = (amax*amax)/std::max(1e-12, jmax);
  if (dv <= dv_tri) {
    const double a_pk = std::sqrt(std::max(0.0, dv*jmax));
    const double t_j  = a_pk / jmax;
    const double s_half = (a_pk*a_pk*a_pk)/(3.0*jmax*jmax);
    H.s = s_half; H.t_j = t_j; H.t_a = 0.0; H.t_dn = t_j;
  } else {
    const double t_j = amax / jmax;
    const double t_a = (dv/amax) - t_j;
    const double s1 = (1.0/6.0)*(amax*amax*amax)/(jmax*jmax);
    const double s2 = 0.5*(amax*amax)*t_a/jmax;
    const double s3 = 0.5*amax*t_a*t_a;
    H.s = s1 + s2 + s3; H.t_j = t_j; H.t_a = t_a; H.t_dn = t_j;
  }
  return H;
}

TrajectoryPlanner::SCurve
TrajectoryPlanner::makeSCurve_L(double L, double v0, double vT,
                                double vmax, double amax, double jmax) {
  SCurve P; P.s0=0.0; P.v0=v0; P.vT=vT; P.vmax=vmax; P.amax=amax; P.jmax=jmax;
  L = std::max(0.0, L);
  const double v_lo = std::max(v0, vT);
  const double v_hi = vmax;

  // Build timings (t_j, t_a, t_dn) for a given dv; H.s is not used anymore.
	auto timings_for_dv = [&](double dv, HalfRes &H){
	dv = std::max(0.0, dv);
	const double dv_tri = (amax*amax)/std::max(1e-12, jmax);
	if (dv <= dv_tri) {
			const double a_pk = std::sqrt(dv * jmax);
			H.t_j  = a_pk / jmax;
			H.t_a  = 0.0;
			H.t_dn = H.t_j;
	} else {
			H.t_j  = amax / jmax;
			H.t_a  = (dv / amax) - H.t_j;
			H.t_dn = H.t_j;
	}
	};

	// Integrate one "half" distance with those timings starting at v_start.
	auto half_distance = [&](double v_start, bool accelerating, const HalfRes &H)->double{
	double a = 0.0, v = v_start, s = 0.0;
	const double j = accelerating ? +jmax : -jmax;
	seg_integ( j, H.t_j,  a, v, s);   // ramp accel up/down
	seg_integ( 0, H.t_a,  a, v, s);   // constant accel (j=0)
	seg_integ(-j, H.t_dn, a, v, s);   // ramp accel back to zero
	return s;
	};

	// Sum accel+decel half distances for a given v_peak
	auto dist_for_vpeak = [&](double vp, double &s_acc, double &s_dec,
													HalfRes &Ha, HalfRes &Hd) {
	const double dv_a = std::max(0.0, vp - v0);
	const double dv_d = std::max(0.0, vp - vT);
	timings_for_dv(dv_a, Ha);
	timings_for_dv(dv_d, Hd);
	s_acc = half_distance(v0, /*accelerating=*/true,  Ha);
	s_dec = half_distance(vp, /*accelerating=*/false, Hd);
	return s_acc + s_dec;
	};


  HalfRes Ha{}, Hd{}; double s_acc=0, s_dec=0;
  (void)dist_for_vpeak(v_hi, s_acc, s_dec, Ha, Hd);
  double t_cruise = (L - (s_acc + s_dec)) / std::max(1e-9, v_hi);

  if (t_cruise >= 0.0) {
    P.v_peak = v_hi;
    P.ta_j = Ha.t_j; P.ta_a = Ha.t_a; P.ta_dn = Ha.t_dn;
    P.td_j = Hd.t_j; P.td_a = Hd.t_a; P.td_dn = Hd.t_dn;
    P.t_cruise = t_cruise;
  } else {
    // Not enough distance to reach vmax → triangular: solve vp
    double lo = v_lo, hi = v_hi;
    for (int it=0; it<40; ++it){
      double mid = 0.5*(lo+hi);
      HalfRes Ha_m,Hd_m; double sa,sd;
      double s_sum = dist_for_vpeak(mid, sa, sd, Ha_m, Hd_m);
      if (s_sum > L) hi = mid; else lo = mid;
    }
    const double vp = 0.5*(lo+hi);
    P.v_peak = vp;
    HalfRes Ha_f,Hd_f; double sa,sd;
    (void)dist_for_vpeak(vp, sa, sd, Ha_f, Hd_f);
    P.ta_j = Ha_f.t_j; P.ta_a = Ha_f.t_a; P.ta_dn = Ha_f.t_dn;
    P.td_j = Hd_f.t_j; P.td_a = Hd_f.t_a; P.td_dn = Hd_f.t_dn;
    P.t_cruise = 0.0;
  }

  P.T = (P.ta_j+P.ta_a+P.ta_dn) + P.t_cruise + (P.td_j+P.td_a+P.td_dn);
  return P;
}

void TrajectoryPlanner::evalSCurve(const SCurve& P, double t,
                                   double& s, double& v, double& a) {
  t = std::max(0.0, std::min(t, P.T));
  s = 0.0; v = P.v0; a = 0.0;
  double tt = t;
  auto step = [&](double j, double dt){
    if (dt<=0) return;
    seg_integ(j, dt, a, v, s);
    tt -= dt;
  };
  // accel half
  const double t1 = std::min(P.ta_j, tt); step(+P.jmax, t1);
  const double t2 = std::min(P.ta_a, tt); step(0.0,    t2);
  const double t3 = std::min(P.ta_dn,tt); step(-P.jmax,t3);
  // cruise
  const double t4 = std::min(P.t_cruise, tt); step(0.0, t4);
  // decel half
  const double t5 = std::min(P.td_j, tt); step(-P.jmax, t5);
  const double t6 = std::min(P.td_a, tt); step( 0.0,    t6);
  const double t7 = std::min(P.td_dn,tt); step(+P.jmax, t7);
}

bool TrajectoryPlanner::plan(double x0, double y0, double z0, double psi0,
                             double dx_goal,
                             double u_term,
                             double vmax_u, double vmax_v,
                             double amax_t,
                             double wmax, double awmax,
                             double rmax, double armax, 
														 double jerk_ratio){
  planned_ = false;

  // cache
  x0_ = x0; y0_ = y0; z0_ = z0; psi0_ = psi0;
  dx_goal_ = -dx_goal; // keep your convention
  u_term_ = std::min(std::max(u_term, 0.0), std::max(1e-9, vmax_u));

  // limits + implied jerks (choose ~0.2 s jerk-up time by default)
  vmax_u_ = vmax_u; vmax_v_ = vmax_v; amax_t_ = amax_t;
  wmax_ = wmax; awmax_ = awmax;
  rmax_ = rmax; armax_ = armax;
  jtrans_ = amax_t_/jerk_ratio;//(amax_t_>0) ? (amax_t_/jerk_ratio) : 1.0;
  jdepth_ = awmax_/jerk_ratio;//(awmax_>0) ? (awmax_/jerk_ratio) : 1.0;
  jyaw_   = armax_/jerk_ratio;//(armax_>0) ? (armax_/jerk_ratio) : 1.0;

  // LOS to (dx_goal_, 0)
  const double gx = dx_goal_, gy = 0.0;
  const double dx = gx - x0_,   dy = gy - y0_;
  L_total_ = std::hypot(dx, dy);
  if (L_total_ < 1e-9) { d_hat_<<1,0; psi_LOS_=0.0; }
  else { d_hat_<< dx/L_total_, dy/L_total_; psi_LOS_ = std::atan2(d_hat_.y(), d_hat_.x()); }

  // Phase 1: yaw-in (S-curve in angle)
  const double dpsi1 = angleDiff(psi_LOS_, psi0_);
  yaw_in_q_ = makeSCurve_L(/*L=|Δψ|*/ std::abs(dpsi1), /*ω0*/0, /*ωT*/0,
                           /*ωmax*/rmax_, /*αmax*/armax_, /*jmax*/jyaw_);
  timeline_.T1 = yaw_in_q_.T;

  // Phase 3 yaw-out profile (build now to get footprint)
  const double dpsi3 = std::abs(angleDiff(0.0, psi_LOS_));
  yaw_out_q_ = makeSCurve_L(/*|Δψ|*/ dpsi3, 0, 0, rmax_, armax_, jyaw_);
  timeline_.T3 = yaw_out_q_.T;

  // Blend footprint with constant surge u_term_
  IxF_ = IyF_ = 0.0;
  {
    const double a = 0.0, b = timeline_.T3;
    const double c1 = 0.5*(b-a), c2 = 0.5*(b+a);
    for (int i=0;i<8;++i){
      const double t1 = c1*(-GL16_x[i]) + c2;
      const double t2 = c1*( GL16_x[i]) + c2;
      double s1,v1,a1, s2,v2,a2;
      evalSCurve(yaw_out_q_, t1, s1, v1, a1);
      evalSCurve(yaw_out_q_, t2, s2, v2, a2);
      const double psi1 = wrapToPi(psi_LOS_ + ( (dpsi3>=0)? +s1 : -s1 ) );
      const double psi2 = wrapToPi(psi_LOS_ + ( (dpsi3>=0)? +s2 : -s2 ) );
      IxF_ += GL16_w[i]*( std::cos(psi1) + std::cos(psi2) );
      IyF_ += GL16_w[i]*( std::sin(psi1) + std::sin(psi2) );
    }
    IxF_ *= c1; IyF_ *= c1;
  }

  // Phase 2 switch to hit y=0 after blend
  const double y_switch = - u_term_ * IyF_;
  double s_switch = 0.0;
  if (std::abs(d_hat_.y()) < 1e-9){
    if (std::abs(y0_ - y_switch) > 1e-6) return false;
    s_switch = 0.0;
  } else {
    s_switch = (y_switch - y0_) / d_hat_.y();
    if (s_switch < 0.0) return false;
  }
  L2_ = s_switch;

  // Phase 2 LOS translation (S-curve distance)
  const double v_pre = std::min(u_term_, vmax_u_);
  los_q_ = makeSCurve_L(/*L*/ L2_, /*v0*/0.0, /*vT*/v_pre,
                        /*vmax*/vmax_u_, /*amax*/amax_t_, /*jmax*/jtrans_);
  timeline_.T2 = los_q_.T;

  // pre-blend state at end of Phase 2
  {
    double s_end, sd_end, sdd_end;
    evalSCurve(los_q_, timeline_.T2, s_end, sd_end, sdd_end);
    P_pre_ = Eigen::Vector2d(x0_, y0_) + d_hat_ * s_end;
    V_pre_ = d_hat_ * sd_end;
    A_pre_ = d_hat_ * sdd_end;
  }

  // end-of-blend x
  x_end_blend_ = P_pre_.x() + u_term_ * IxF_;

  // Phase 4 to origin along x
  const double rem_to_zero = 0.0 - x_end_blend_;
  tail_dir_ = (rem_to_zero > 1e-9) ? +1.0 : (rem_to_zero < -1e-9 ? -1.0 : 0.0);
  timeline_.T4 = (tail_dir_==0.0)? 0.0 : std::abs(rem_to_zero)/u_term_;

  // Depth S-curve from z0 -> 0 (we use magnitude and sign outside)
  depth_q_ = makeSCurve_L(/*L*/ std::abs(z0_), /*v0*/0, /*vT*/0,
                          /*vmax*/ std::abs(wmax_), /*amax*/ std::abs(awmax_), /*jmax*/ std::abs(jdepth_));
  timeline_.Tz = depth_q_.T;

  // anchors
  timeline_.t0_yaw_in = 0.0;
  timeline_.t0_los    = timeline_.T1;
  timeline_.t0_blend  = timeline_.T1 + timeline_.T2;
  timeline_.t0_final  = timeline_.t0_blend + timeline_.T3;
  timeline_.T_total   = std::max(timeline_.t0_final + timeline_.T4, timeline_.Tz);

  planned_ = true;
  return true;
}

void TrajectoryPlanner::evaluate(double t,
                                 double& x, double& y, double& z,
                                 double& xd, double& yd, double& zd,
                                 double& xdd, double& ydd, double& zdd,
                                 double& yaw, double& r, double& ar) const {
  // defaults
  x=x0_; y=y0_; z=z0_;
  xd=yd=zd=0.0;
  xdd=ydd=zdd=0.0;
  yaw=psi0_; r=ar=0.0;

  if (!planned_) return;
  const auto& tl = timeline_;

  // Depth runs independently from t=0 (map |z| profile onto sign to go to 0)
  {
    double sz,vz,az;
    evalSCurve(depth_q_, clamp01(t, tl.Tz), sz, vz, az);
    const double signz = (z0_>=0.0)? -1.0 : +1.0; // move toward zero
    z   = z0_ + signz * sz;
    zd  = signz * vz;
    zdd = signz * az;
  }

  // Phase 1: yaw-in
  if (t <= tl.t0_los){
    double s_rel, v_rel, a_rel;
    evalSCurve(yaw_in_q_, t - tl.t0_yaw_in, s_rel, v_rel, a_rel);
    const double sign = (angleDiff(psi_LOS_, psi0_)>=0)? +1.0 : -1.0;
    yaw = wrapToPi(psi0_ + sign*s_rel);
    r   = sign*v_rel;
    ar  = sign*a_rel;
    return;
  }

  // Phase 2: LOS translation (yaw fixed at psi_LOS_)
  if (t <= tl.t0_blend){
    const double tau = t - tl.t0_los;
    double s,v,a; evalSCurve(los_q_, tau, s, v, a);
    x   = x0_ + d_hat_.x()*s;
    y   = y0_ + d_hat_.y()*s;
    xd  = d_hat_.x()*v;
    yd  = d_hat_.y()*v;
    xdd = d_hat_.x()*a;
    ydd = d_hat_.y()*a;
    yaw = wrapToPi(psi_LOS_);
    r = 0.0; ar = 0.0;
    return;
  }

  // Phase 3: constant-speed blend (surge u_term_, yaw S-curve from psi_LOS->0)
  if (t <= tl.t0_final){
    const double tau = t - tl.t0_blend;
    double s_rel, v_rel, a_rel;
    evalSCurve(yaw_out_q_, tau, s_rel, v_rel, a_rel);
    const double sign = (angleDiff(0.0, psi_LOS_)>=0)? +1.0 : -1.0;
    const double psi  = wrapToPi(psi_LOS_ + sign*s_rel);
    yaw = psi; r = sign*v_rel; ar = sign*a_rel;

    // position = P_pre + ∫ u [cosψ, sinψ] dt  (integrate to current tau)
    double Ix=0.0, Iy=0.0;
    const double a=0.0, b=tau;
    const double c1 = 0.5*(b-a), c2 = 0.5*(b+a);
    for (int i=0;i<8;++i){
      const double t1 = c1*(-GL16_x[i]) + c2;
      const double t2 = c1*( GL16_x[i]) + c2;
      double s1,v1,a1, s2,v2,a2;
      evalSCurve(yaw_out_q_, t1, s1, v1, a1);
      evalSCurve(yaw_out_q_, t2, s2, v2, a2);
      const double psi1 = wrapToPi(psi_LOS_ + sign*s1);
      const double psi2 = wrapToPi(psi_LOS_ + sign*s2);
      Ix += GL16_w[i]*( std::cos(psi1) + std::cos(psi2) );
      Iy += GL16_w[i]*( std::sin(psi1) + std::sin(psi2) );
    }
    Ix *= c1; Iy *= c1;
    const Eigen::Vector2d dP = u_term_ * Eigen::Vector2d(Ix, Iy);
    const Eigen::Vector2d P = P_pre_ + dP;
    const Eigen::Vector2d V = u_term_ * Eigen::Vector2d(std::cos(psi), std::sin(psi));
    const Eigen::Vector2d A = u_term_ * (sign*v_rel) * Eigen::Vector2d(-std::sin(psi), std::cos(psi));

    x=P.x(); y=P.y();
    xd=V.x(); yd=V.y();
    xdd=A.x(); ydd=A.y();
    return;
  }

  // Phase 4: final straight to origin along x
  {
    const double tau = t - tl.t0_final;
    x   = x_end_blend_ + tail_dir_ * u_term_ * clamp01(tau, tl.T4);
    y   = 0.0;
    xd  = (tl.T4>0.0? tail_dir_ * u_term_ : 0.0);
    yd  = 0.0;
    xdd = ydd = 0.0;
    yaw = 0.0; r = 0.0; ar = 0.0;
  }
}
