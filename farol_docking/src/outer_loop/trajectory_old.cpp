#include <farol_docking/outer_loop/trajectory.hpp>  



// 16-pt Gauss–Legendre nodes/weights for [-1,1]
static const double GL16_x[8] = {
  0.0950125098376374, 0.2816035507792589, 0.4580167776572274, 0.6178762444026438,
  0.7554044083550030, 0.8656312023878318, 0.9445750230732326, 0.9894009349916499
};
static const double GL16_w[8] = {
  0.1894506104550685, 0.1826034150449236, 0.1691565193950025, 0.1495959888165767,
  0.1246289712555339, 0.0951585116824928, 0.0622535239386479, 0.0271524594117541
};
// --------- local helpers ---------
static inline double clamp01(double t, double T) {
  if (t < 0.0) return 0.0;
  if (t > T)   return T;
  return t;
}

// Wrap angle to (-pi, pi]
static inline double wrapToPi(double a) {
  double r = std::fmod(a + M_PI, 2.0*M_PI);
  if (r < 0) r += 2.0*M_PI;
  return r - M_PI;
}

// Shortest signed difference: to - from, wrapped to (-pi, pi]
static inline double angleDiff(double to, double from) {
  return wrapToPi(to - from);
}


// ====================
// TrajectoryPlanner
// ====================

TrajectoryPlanner::TrajectoryPlanner() = default;

// -------------------- Quintic utilities --------------------

TrajectoryPlanner::Quintic
TrajectoryPlanner::makeQuintic(double s0, double v0, double a0,
                               double sT, double vT, double aT, double T) const
{
  Quintic q;
  q.c0 = s0;
  q.c1 = v0;
  q.c2 = 0.5 * a0;
  q.T  = T;

  Eigen::Matrix3d M;
  M << std::pow(T,3), std::pow(T,4), std::pow(T,5),
       3*std::pow(T,2), 4*std::pow(T,3), 5*std::pow(T,4),
       6*T, 12*std::pow(T,2), 20*std::pow(T,3);

  Eigen::Vector3d b;
  b << sT - (q.c0 + q.c1*T + q.c2*T*T),
       vT - (q.c1 + 2*q.c2*T),
       aT - (2*q.c2);

  Eigen::Vector3d x = M.colPivHouseholderQr().solve(b);
  q.c3 = x(0); q.c4 = x(1); q.c5 = x(2);
  return q;
}

void
TrajectoryPlanner::evalQuintic(const Quintic& q, double t,
                               double& s, double& v, double& a) const
{
  t = clamp01(t, q.T);
  const double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
  s = q.c0 + q.c1*t + q.c2*t2 + q.c3*t3 + q.c4*t4 + q.c5*t5;
  v = q.c1 + 2*q.c2*t + 3*q.c3*t2 + 4*q.c4*t3 + 5*q.c5*t4;
  a = 2*q.c2 + 6*q.c3*t + 12*q.c4*t2 + 20*q.c5*t3;
}

// -------------------- Closed-form trapezoid timing --------------------

TrajectoryPlanner::TrapTime
TrajectoryPlanner::trapezoid_time(double L, double v0, double vT,
                                  double vmax, double amax) const
{
  TrapTime out;
  L  = std::abs(L);
  v0 = std::abs(v0);
  vT = std::abs(vT);

  // Candidate triangular peak speed
  const double vp2 = 0.5*(v0*v0 + vT*vT) + amax*L;
  const double vp  = std::sqrt(std::max(0.0, vp2));

  if (vp <= vmax + 1e-12) {
    // Triangular (no cruise)
    out.triangular = true;
    out.v_peak = vp;
    out.T_acc = (vp - v0)/amax;
    out.T_dec = (vp - vT)/amax;
    out.T_cruise = 0.0;
    out.T = out.T_acc + out.T_dec;
  } else {
    // Trapezoidal (with cruise at vmax)
    out.triangular = false;
    out.v_peak = vmax;
    out.T_acc = (vmax - v0)/amax;
    out.T_dec = (vmax - vT)/amax;

    const double L_ad =
      (vmax*vmax - v0*v0)/(2*amax) +
      (vmax*vmax - vT*vT)/(2*amax);

    const double L_cruise = std::max(0.0, L - L_ad);
    out.T_cruise = L_cruise / std::max(1e-9, vmax);
    out.T = out.T_acc + out.T_cruise + out.T_dec;
  }
  return out;
}

// -------------------- Plan full trajectory --------------------

bool TrajectoryPlanner::plan(double x0, double y0, double z0, double psi0,
                        double dx_goal,
                        double u_term,
                        double vmax_u, double vmax_v,
                        double amax_t,
                        double wmax, double awmax,
                        double rmax, double armax)
{

  ROS_INFO_STREAM("u_term: " <<u_term);
  ROS_INFO_STREAM("vmax_u: " <<vmax_u);
  ROS_INFO_STREAM("vmax_v: " <<vmax_v);
  ROS_INFO_STREAM("amax_t: " <<amax_t);
  ROS_INFO_STREAM("wmax: " <<wmax);
  ROS_INFO_STREAM("awmax: " <<awmax);
  ROS_INFO_STREAM("rmax: " <<rmax);
  ROS_INFO_STREAM("armax: " <<armax);
  // -------------------------
  // Cache start + targets
  // -------------------------
  x0_ = x0; y0_ = y0; z0_ = z0; psi0_ = psi0;
  dx_goal_ = -dx_goal;
  u_term_ = std::min(std::max(u_term, 0.0), std::max(1e-9, vmax_u));

  // LOS direction to the homing line point (dx_goal, 0)
  const double gx = dx_goal_, gy = 0.0;
  const double dx = gx - x0_,   dy = gy - y0_;
  L_total_ = std::hypot(dx, dy);
  if (L_total_ < 1e-6) {
    d_hat_ << 1.0, 0.0;
    psi_LOS_ = 0.0;
  } else {
    d_hat_ << dx / L_total_, dy / L_total_;
    psi_LOS_ = std::atan2(d_hat_.y(), d_hat_.x());
  }

  // -------------------------
  // Phase 1: yaw-in (psi0_ -> psi_LOS_)
  // -------------------------

   // ===== Phase 1: yaw-in (psi0_ -> psi_LOS_) =====
	const double dpsi1 = angleDiff(psi_LOS_, psi0_);
	const TrapTime Tyaw = trapezoid_time(std::abs(dpsi1),
																			/*v0=*/0.0, /*vT=*/0.0,
																			/*vmax=*/rmax, /*amax=*/armax);
	timeline_.T1  = std::max(0.3, Tyaw.T);
	// Build RELATIVE yaw profile: 0 -> dpsi1
	yaw_in_q_     = makeQuintic(/*s0=*/0.0, 0, 0,
															/*sT=*/dpsi1, 0, 0,
															timeline_.T1);

  // -------------------------
  // Phase 2: LOS translation (we'll FINALIZE its length after we compute the blend footprint)
  // Build a provisional LOS just to get a consistent pre-blend boundary; we will rebuild it later.
  // -------------------------
  // Provisional: aim to cover the straight-line distance at end speed v_pre
  const double v_pre = std::min(u_term_, vmax_u);
  L2_ = L_total_; // provisional LOS length (will be replaced below)
  const TrapTime Tlos0 = trapezoid_time(L2_, /*v0=*/0.0, /*vT=*/v_pre,
                                        /*vmax=*/vmax_u, /*amax=*/amax_t);
  timeline_.T2 = std::max(0.5, Tlos0.T);
  los_q_ = makeQuintic(/*s0=*/0.0, /*v0=*/0.0, /*a0=*/0.0,
                       /*sT=*/L2_, /*vT=*/v_pre, /*aT=*/0.0,
                       timeline_.T2);

  // End state of provisional Phase 2 → pre-blend boundary
  {
    double s_end, sd_end, sdd_end;
    evalQuintic(los_q_, timeline_.T2, s_end, sd_end, sdd_end);
    P_pre_ = Eigen::Vector2d(x0_, y0_) + d_hat_ * s_end;
    V_pre_ = d_hat_ * sd_end;
    A_pre_ = d_hat_ * sdd_end;
  }

  // -------------------------
  // Phase 3: yaw-out (psi_LOS_ -> 0) over T3 with zero rates/accels
  // (duration from yaw limits; we will use this T3 to compute the blend footprint)
  // -------------------------
  const double dpsi2 = angleDiff(/*to=*/0.0, /*from=*/psi_LOS_);
  const TrapTime Tyaw2 = trapezoid_time(std::abs(dpsi2),
                                        /*v0=*/0.0, /*vT=*/0.0,
                                        /*vmax=*/rmax, /*amax=*/armax);
  timeline_.T3 = std::max(0.5, Tyaw2.T);
  yaw_out_q_ = makeQuintic(/*s0=*/0.0, /*v0=*/0.0, /*a0=*/0.0,
                           /*sT=*/dpsi2, /*vT=*/0.0, /*aT=*/0.0,
                           timeline_.T3);

  // -------------------------
  // NEW: Compute full-blend footprint Δr_b = ∫_0^{T3} ū [cos ψ(t), sin ψ(t)] dt
  // where ψ(t) = ψ_LOS_ + s_rel(t), s_rel is yaw_out_q_ (0→dpsi2).
  // -------------------------
  IxF_ = 0.0; IyF_ = 0.0;
  {
    const double aF = 0.0, bF = timeline_.T3;
    const double c1F = 0.5 * (bF - aF), c2F = 0.5 * (bF + aF);
    for (int i = 0; i < 8; ++i) {
      const double t1 = c1F * (-GL16_x[i]) + c2F;
      const double t2 = c1F * ( GL16_x[i]) + c2F;
      double s1,v1,a1, s2,v2,a2;
      evalQuintic(yaw_out_q_, t1, s1, v1, a1);
      evalQuintic(yaw_out_q_, t2, s2, v2, a2);
      IxF_ += GL16_w[i] * ( std::cos(wrapToPi(psi_LOS_ + s1)) + std::cos(wrapToPi(psi_LOS_ + s2)) );
      IyF_ += GL16_w[i] * ( std::sin(wrapToPi(psi_LOS_ + s1)) + std::sin(wrapToPi(psi_LOS_ + s2)) );
    }
    IxF_ *= 0.5 * timeline_.T3;   // ∫ cos ψ dt
    IyF_ *= 0.5 * timeline_.T3;   // ∫ sin ψ dt
  }


  // -------------------------
  // Retarget Phase 2 switch so that Phase 3 lands on y=0:
  // require y_pre = - ū * IyF  ⇒  s_switch = (y_pre - y0_)/d_hat_.y()
  // -------------------------
  const double y_switch = 0.0 - u_term_ * IyF_;
  double s_switch = 0.0;
  if (std::abs(d_hat_.y()) > 1e-9) {
    s_switch = (y_switch - y0_) / d_hat_.y();
  } else {
    // LOS parallel to x-axis: only feasible if already at y_switch
    if (std::abs(y0_ - y_switch) > 1e-6) return false;
    s_switch = 0.0;
  }
  if (s_switch < 0.0) return false; // switch would be behind start

  // Rebuild Phase 2 to end exactly at this switch point
  L2_ = s_switch;
  const TrapTime Tlos = trapezoid_time(L2_, /*v0=*/0.0, /*vT=*/v_pre,
                                       /*vmax=*/vmax_u, /*amax=*/amax_t);
  timeline_.T2 = std::max(0.5, Tlos.T);
  los_q_ = makeQuintic(/*s0=*/0.0, /*v0=*/0.0, /*a0=*/0.0,
                       /*sT=*/L2_, /*vT=*/v_pre, /*aT=*/0.0,
                       timeline_.T2);

  // Refresh the pre-blend boundary now that L2_ is final
  {
    double s_end, sd_end, sdd_end;
    evalQuintic(los_q_, timeline_.T2, s_end, sd_end, sdd_end);
    P_pre_ = Eigen::Vector2d(x0_, y0_) + d_hat_ * s_end;
    V_pre_ = d_hat_ * sd_end;
    A_pre_ = d_hat_ * sdd_end;
  }
  x_end_blend_ = P_pre_.x() + u_term_ * IxF_;

  // -------------------------
  // Phase 4: time to reach x = dx_goal at speed ū after the blend
  // end-of-blend x is: x_end = P_pre_.x + ū * IxF
  // -------------------------
  const double rem_to_zero = 0.0 - x_end_blend_;
  timeline_.T4 = std::abs(rem_to_zero) / u_term_;

  // -------------------------
  // Depth (unchanged): z0 → 0
  // -------------------------
  const TrapTime Tz = trapezoid_time(z0_ - 0.0, 0.0, 0.0, /*vmax=*/wmax, /*amax=*/awmax);
  timeline_.Tz   = std::max(0.3, Tz.T);
  depth_q_       = makeQuintic(z0_, 0, 0, 0.0, 0, 0, timeline_.Tz);

  timeline_.t0_yaw_in = 0.0;
  timeline_.t0_los    = timeline_.T1;
  timeline_.t0_blend  = timeline_.T1 + timeline_.T2;
  timeline_.t0_final  = timeline_.t0_blend + timeline_.T3;
  timeline_.T_total   = std::max(timeline_.t0_final + timeline_.T4, timeline_.Tz);

  planned_ = true;
  return true;
}

// -------------------- Evaluate trajectory --------------------

void TrajectoryPlanner::evaluate(double t,
                            double& x, double& y, double& z,
                            double& xd, double& yd, double& zd,
                            double& xdd, double& ydd, double& zdd,
                            double& yaw, double& r, double& ar) const
{
  // Defaults
  x = x0_; y = y0_; z = z0_;
  xd = yd = zd = 0.0;
  xdd = ydd = zdd = 0.0;
  yaw = psi0_; r = ar = 0.0;

  if (!planned_) return;

  const auto& tl = timeline_;

  // Depth runs independently from t=0
  {
    double sz, vz, az;
    evalQuintic(depth_q_, clamp01(t, tl.Tz), sz, vz, az);
    z   = sz;
    zd  = vz;
    zdd = az;
  }

  // Phase 1: yaw-in
	if (t <= tl.T1) {
		double s_rel, v_rel, a_rel;
		evalQuintic(yaw_in_q_, t, s_rel, v_rel, a_rel);
		yaw = wrapToPi(psi0_ + s_rel);  // rebase + wrap (wrap optional for quaternion)
		r   = v_rel;
		ar  = a_rel;
		return;
	}

  // Phase 2: LOS translation (yaw fixed at psi_LOS_)
  if (t <= tl.t0_blend) {
    const double tau = t - tl.t0_los;
    double s,v,a;
    evalQuintic(los_q_, tau, s, v, a);

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

  // Phase 3: planar blend + yaw-out
  if (t <= tl.t0_final) {
    const double tau = t - tl.t0_blend;

    // --- Yaw: existing relative quintic (psi_LOS_ -> 0) ---
    double s_rel, v_rel, a_rel;
    evalQuintic(yaw_out_q_, tau, s_rel, v_rel, a_rel);
    const double psi = wrapToPi(psi_LOS_ + s_rel);
    yaw = psi; r = v_rel; ar = a_rel;

    // --- Position: constant-speed ū following yaw(t): P = P_pre_ + ∫ ū [cos psi, sin psi] dt ---
    double Ix = 0.0, Iy = 0.0;
    const double a = 0.0, b = tau;
    const double c1 = 0.5*(b-a), c2 = 0.5*(b+a);
    for (int i=0;i<8;++i){
      const double t1 = c1*(-GL16_x[i]) + c2;
      const double t2 = c1*( GL16_x[i]) + c2;
      double s1, v1, a1, s2, v2, a2;
      evalQuintic(yaw_out_q_, t1, s1, v1, a1);
      evalQuintic(yaw_out_q_, t2, s2, v2, a2);
      Ix += GL16_w[i] * ( std::cos(wrapToPi(psi_LOS_ + s1)) + std::cos(wrapToPi(psi_LOS_ + s2)) );
      Iy += GL16_w[i] * ( std::sin(wrapToPi(psi_LOS_ + s1)) + std::sin(wrapToPi(psi_LOS_ + s2)) );
    }
    Ix *= c1; Iy *= c1;
    const Eigen::Vector2d dP = u_term_ * Eigen::Vector2d(Ix, Iy);

    const Eigen::Vector2d P = P_pre_ + dP;
    const Eigen::Vector2d V = u_term_ * Eigen::Vector2d(std::cos(psi), std::sin(psi));
    const Eigen::Vector2d A = u_term_ * v_rel * Eigen::Vector2d(-std::sin(psi), std::cos(psi));

    x = P.x(); y = P.y();
    xd = V.x(); yd = V.y();
    xdd = A.x(); ydd = A.y();
    return;
  }

  // Phase 4: final constant-speed along x, yaw=0
  {
    const double tau = t - tl.t0_final;
    x   = x_end_blend_ + u_term_ * clamp01(tau, tl.T4);
    xd  = u_term_;
    xdd = 0.0; 
    
    y   = 0.0;
    yd  = 0.0;
    ydd = 0.0;

    yaw = 0.0; 
    r = 0.0; 
    ar = 0.0;

  }
}
