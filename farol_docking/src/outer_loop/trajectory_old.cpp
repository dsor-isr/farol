#include <farol_docking/outer_loop/trajectory.hpp>  


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
  // Cache start + targets
  x0_ = x0; y0_ = y0; z0_ = z0; psi0_ = psi0;
  dx_goal_ = -dx_goal; u_term_ = u_term;

  // LOS geometry to homing point [dx_goal, 0]
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

	

  // ===== Phase 2: LOS straight to pre-blend =====
  const double Lb = std::max(2.0, 0.15 * L_total_); // reserve segment for blend
  L2_ = std::max(0.0, L_total_ - Lb);
  const double v_pre = std::min(u_term_, vmax_u);
  const TrapTime Tlos = trapezoid_time(L2_, /*v0=*/0.0, /*vT=*/v_pre,
                                       /*vmax=*/vmax_u, /*amax=*/amax_t);
  timeline_.T2 = std::max(0.5, Tlos.T);
  los_q_ = makeQuintic(0, 0, 0, L2_, v_pre, 0, timeline_.T2);

  // End state of Phase 2 → pre-blend boundary
  {
    double s_end, sd_end, sdd_end;
    evalQuintic(los_q_, timeline_.T2, s_end, sd_end, sdd_end);
    P_pre_ = Eigen::Vector2d(x0_, y0_) + d_hat_ * s_end;
    V_pre_ = d_hat_ * sd_end;
    A_pre_ = d_hat_ * sdd_end;
  }

  // ===== Phase 3: planar blend to homing; terminal v=[dir*u_term, 0], a=0 =====
  const double dir = (dx_goal_ >= 0.0) ? -1.0 : +1.0;
  const Eigen::Vector2d v_end(dir * u_term_, 0.0);
  const Eigen::Vector2d a_end(0.0, 0.0);

  const TrapTime Tx = trapezoid_time(dx_goal_ - P_pre_.x(),
                                     V_pre_.x(), v_end.x(),
                                     /*vmax=*/vmax_u, /*amax=*/amax_t);
  const TrapTime Ty = trapezoid_time(0.0 - P_pre_.y(),
                                     V_pre_.y(), v_end.y(),
                                     /*vmax=*/vmax_v, /*amax=*/amax_t);
  timeline_.T3 = std::max({0.5, Tx.T, Ty.T});

  blend_x_q_ = makeQuintic(P_pre_.x(), V_pre_.x(), A_pre_.x(),
                           dx_goal_,   v_end.x(),  0.0, timeline_.T3);
  blend_y_q_ = makeQuintic(P_pre_.y(), V_pre_.y(), A_pre_.y(),
                           0.0,        v_end.y(),  0.0, timeline_.T3);

	// ===== Phase 3: yaw-out (psi_LOS_ -> 0) over same T3 =====
	const double dpsi2 = angleDiff(/*to=*/0.0, /*from=*/psi_LOS_);
	const TrapTime Tyaw2 = trapezoid_time(std::abs(dpsi2),
																			0.0, 0.0, rmax, armax);
	if (Tyaw2.T > timeline_.T3) {
	// stretch planar blend to match yaw time
	timeline_.T3 = Tyaw2.T;
	blend_x_q_ = makeQuintic(P_pre_.x(), V_pre_.x(), A_pre_.x(),
													dx_goal_, v_end.x(), 0.0, timeline_.T3);
	blend_y_q_ = makeQuintic(P_pre_.y(), V_pre_.y(), A_pre_.y(),
													0.0,      v_end.y(), 0.0, timeline_.T3);
	}
	// Build RELATIVE yaw profile: 0 -> dpsi2
	yaw_out_q_ = makeQuintic(/*s0=*/0.0, 0, 0,
													/*sT=*/dpsi2, 0, 0,
													timeline_.T3);

  // ===== Phase 4: final constant-velocity along x to 0 (yaw=0) =====
  const double L4 = std::abs(dx_goal_ - 0.0);
  timeline_.T4 = (u_term_ > 1e-6) ? (L4 / u_term_) : 0.0;

  // ===== Depth: z0_ → 0 (independent) =====
  const TrapTime Tz = trapezoid_time(z0_ - 0.0, 0.0, 0.0, /*vmax=*/wmax, /*amax=*/awmax);
  timeline_.Tz   = std::max(0.3, Tz.T);
  depth_q_       = makeQuintic(z0_, 0, 0, 0.0, 0, 0, timeline_.Tz);

  // Offsets & total time
  timeline_.t0_yaw_in = 0.0;
  timeline_.t0_los    = timeline_.T1;
  timeline_.t0_blend  = timeline_.T1 + timeline_.T2;
  timeline_.t0_final  = timeline_.T1 + timeline_.T2 + timeline_.T3;
  timeline_.T_total   = timeline_.T1 + timeline_.T2 + timeline_.T3 + timeline_.T4;

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

    double sx,vx,ax, sy,vy,ay;
    evalQuintic(blend_x_q_, tau, sx, vx, ax);
    evalQuintic(blend_y_q_, tau, sy, vy, ay);

    x = sx; y = sy;
    xd = vx; yd = vy;
    xdd = ax; ydd = ay;

    double s_rel, v_rel, a_rel;
		evalQuintic(yaw_out_q_, tau, s_rel, v_rel, a_rel);
		yaw = wrapToPi(psi_LOS_ + s_rel);
		r   = v_rel;
		ar  = a_rel;
		return;
  }

  // Phase 4: final constant-speed along x, yaw=0
  {
    const double tau = t - tl.t0_final;
    const double dir = (dx_goal_ >= 0.0) ? -1.0 : +1.0;

    x   = dx_goal_ + dir * u_term_ * clamp01(tau, tl.T4);
    y   = 0.0;
    xd  = dir * u_term_;
    yd  = 0.0;
    xdd = 0.0; ydd = 0.0;

    yaw = 0.0; r = 0.0; ar = 0.0;
  }
}
