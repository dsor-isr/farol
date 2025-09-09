#include <farol_docking/outer_loop/trajectory.hpp>  
#include <cassert>

// ===== ctor =====
TrajectoryPlanner::TrajectoryPlanner() = default;

// ===== math utils =====
TrajectoryPlanner::Quintic
TrajectoryPlanner::makeQuintic(double s0, double v0, double a0,
                               double sT, double vT, double aT, double T) const {
  Quintic q; q.c0=s0; q.c1=v0; q.c2=0.5*a0; q.T=T;
  Eigen::Matrix3d M;
  M << std::pow(T,3), std::pow(T,4), std::pow(T,5),
       3*std::pow(T,2), 4*std::pow(T,3), 5*std::pow(T,4),
       6*T, 12*std::pow(T,2), 20*std::pow(T,3);
  Eigen::Vector3d b;
  b << sT - (q.c0 + q.c1*T + q.c2*T*T),
       vT - (q.c1 + 2*q.c2*T),
       aT - (2*q.c2);
  Eigen::Vector3d x = M.colPivHouseholderQr().solve(b);
  q.c3=x(0); q.c4=x(1); q.c5=x(2);
  return q;
}

void TrajectoryPlanner::evalQuintic(const Quintic& q, double t,
                                    double& s, double& v, double& a) const {
  t = clamp01(t, q.T);
  const double t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
  s = q.c0 + q.c1*t + q.c2*t2 + q.c3*t3 + q.c4*t4 + q.c5*t5;
  v = q.c1 + 2*q.c2*t + 3*q.c3*t2 + 4*q.c4*t3 + 5*q.c5*t4;
  a = 2*q.c2 + 6*q.c3*t + 12*q.c4*t2 + 20*q.c5*t3;
}

// 1D trapezoid time (kept for depth / optional straights)
TrajectoryPlanner::TrapTime
TrajectoryPlanner::trapezoid_time(double L, double v0, double vT,
                                  double vmax, double amax) const {
  TrapTime out;
  L  = std::abs(L);
  v0 = std::abs(v0);
  vT = std::abs(vT);
  const double vp2 = 0.5*(v0*v0 + vT*vT) + amax*L;
  const double vp  = std::sqrt(std::max(0.0, vp2));
  if (vp <= vmax + 1e-12) {
    out.triangular=true; out.v_peak=vp;
    out.T_acc=(vp - v0)/amax; out.T_dec=(vp - vT)/amax; out.T=out.T_acc+out.T_dec;
  } else {
    out.triangular=false; out.v_peak=vmax;
    out.T_acc=(vmax - v0)/amax; out.T_dec=(vmax - vT)/amax;
    const double L_ad = (vmax*vmax - v0*v0)/(2*amax) + (vmax*vmax - vT*vT)/(2*amax);
    const double L_cruise = std::max(0.0, L - L_ad);
    out.T_cruise = L_cruise / std::max(1e-9, vmax);
    out.T = out.T_acc + out.T_cruise + out.T_dec;
  }
  return out;
}

// ===== integrators (no Fresnel) =====
void TrajectoryPlanner::integrateClothoid(double& x, double& y, double& psi,
                                          double k0, double sigma, double L, double ds,
                                          std::vector<Sample>* path) const {
  if (L <= 1e-12) return;
  const int N = std::max(1, (int)std::ceil(L/ds));
  const double s0 = (path && !path->empty()) ? path->back().s : 0.0;
  double s = 0.0;
  for (int i=1;i<=N;++i) {
    const double ds_i = L / N;
    const double s_mid = s + 0.5*ds_i;
    const double k_mid = k0 + sigma * s_mid; // linear curvature ramp
    if (std::abs(k_mid) < 1e-10) {
      x   += ds_i * std::cos(psi);
      y   += ds_i * std::sin(psi);
      // psi unchanged
    } else {
      const double dpsi = k_mid * ds_i;
      const double R    = 1.0 / k_mid;
      const double cx   = x - R * std::sin(psi);
      const double cy   = y + R * std::cos(psi);
      psi += dpsi;
      x = cx + R * std::sin(psi);
      y = cy - R * std::cos(psi);
    }
    s += ds_i;
    if (path) {
      Sample sm;
      sm.s = s0 + s;
      sm.x = x; sm.y = y; sm.psi = psi; sm.kappa = k_mid;
      path->push_back(sm);
    }
  }
}

void TrajectoryPlanner::integrateArc(double& x, double& y, double& psi,
                                     double kappa, double L, double ds,
                                     std::vector<Sample>* path) const {
  if (L <= 1e-12) return;
  const int N = std::max(1, (int)std::ceil(L/ds));
  const double s0 = (path && !path->empty()) ? path->back().s : 0.0;
  double s = 0.0;
  for (int i=1;i<=N;++i) {
    const double ds_i = L / N;
    if (std::abs(kappa) < 1e-10) {
      x   += ds_i * std::cos(psi);
      y   += ds_i * std::sin(psi);
    } else {
      const double dpsi = kappa * ds_i;
      const double R    = 1.0 / kappa;
      const double cx   = x - R * std::sin(psi);
      const double cy   = y + R * std::cos(psi);
      psi += dpsi;
      x = cx + R * std::sin(psi);
      y = cy - R * std::cos(psi);
    }
    s += ds_i;
    if (path) {
      Sample sm;
      sm.s = s0 + s;
      sm.x = x; sm.y = y; sm.psi = psi; sm.kappa = kappa;
      path->push_back(sm);
    }
  }
}

// CAC path sampling
void TrajectoryPlanner::buildCAC(double x0, double y0, double psi0,
                                 double kR, double sigma, double Lc, double La,
                                 std::vector<Sample>& path, double ds) const {
  if (path.empty()) {
    Sample s0; s0.s=0; s0.x=x0; s0.y=y0; s0.psi=psi0; s0.kappa=0.0;
    path.push_back(s0);
  }
  double x = path.back().x, y = path.back().y, psi = path.back().psi;
  // 1) Clothoid in: kappa(s)=k0 + sigma*s, k0=0 → ramp to kR over Lc
  integrateClothoid(x, y, psi, /*k0=*/0.0, sigma, Lc, ds, &path);
  // 2) Arc: constant curvature kR over La
  integrateArc(x, y, psi, kR, La, ds, &path);
  // 3) Clothoid out: kappa(s)=kR + (-sigma)*s to 0 over Lc
  integrateClothoid(x, y, psi, /*k0=*/kR, /*sigma=*/-sigma, Lc, ds, &path);
}

// CAC net displacement (no sampling)
TrajectoryPlanner::CACResult
TrajectoryPlanner::cac_displacement(double kR, double sigma, double dpsi_total, double ds) const {
  CACResult res;
  const double kabs = std::abs(kR), sabs = std::abs(sigma);
  res.Lc = (sabs > 1e-12) ? (kabs / sabs) : 0.0;
  res.La = std::max(0.0, (std::abs(dpsi_total) / std::max(1e-12,kabs)) - res.Lc);

  // Integrate from origin, heading=0
  double x=0.0, y=0.0, psi=0.0;
  const double sgn = (dpsi_total >= 0.0) ? 1.0 : -1.0;
  const double kR_sgn = sgn * kabs;
  const double sigma_sgn = sgn * sabs;

  // clothoid in
  integrateClothoid(x, y, psi, /*k0=*/0.0, sigma_sgn, res.Lc, ds, nullptr);
  // arc
  integrateArc(x, y, psi, /*kappa=*/kR_sgn, res.La, ds, nullptr);
  // clothoid out
  integrateClothoid(x, y, psi, /*k0=*/kR_sgn, /*-sigma*/ -sigma_sgn, res.Lc, ds, nullptr);

  res.dx = x; res.dy = y; res.dpsi = psi;
  return res;
}

// Straight segment
void TrajectoryPlanner::buildStraight(double x0, double y0, double psi, double L,
                                      std::vector<Sample>& path, double ds) const {
  if (L <= 1e-9) return;
  int N = std::max(1, (int)std::ceil(L/ds));
  double s0 = path.empty()? 0.0 : path.back().s;
  for (int i=1;i<=N;++i) {
    double si = (double)i/N * L;
    Sample sm;
    sm.s    = s0 + si;
    sm.x    = x0 + si*std::cos(psi);
    sm.y    = y0 + si*std::sin(psi);
    sm.psi  = psi;
    sm.kappa= 0.0;
    path.push_back(sm);
  }
}

// ===== time parameterization (constant surge) =====
void TrajectoryPlanner::parameterizeTimeConstantSpeed(const std::vector<Sample>& path,
                                                      double u_term,
                                                      std::vector<double>& t_of_s) const {
  const int N = (int)path.size();
  t_of_s.assign(N, 0.0);
  if (N==0) return;
  t_of_s[0] = 0.0;
  for (int i=1;i<N;++i) {
    double ds = path[i].s - path[i-1].s;
    t_of_s[i] = t_of_s[i-1] + ds / std::max(1e-6, u_term);
  }
}

// ===== depth =====
void TrajectoryPlanner::planDepth(double z0, double wmax, double awmax) {
  TrapTime Tz = trapezoid_time(z0-0.0, 0.0, 0.0, wmax, awmax);
  Tz_ = std::max(0.3, Tz.T);
  depth_q_ = makeQuintic(z0, 0, 0, 0.0, 0, 0, Tz_);
}
void TrajectoryPlanner::evalDepth(double t, double& z, double& zd, double& zdd) const {
  double s,v,a; evalQuintic(depth_q_, clamp01(t, Tz_), s,v,a);
  z=s; zd=v; zdd=a;
}

// ===== PLAN =====
bool TrajectoryPlanner::plan(double x0, double y0, double z0, double psi0,
                             double dx_goal,
                             double u_term,
                             double vmax_u, double vmax_v,
                             double amax_t,
                             double wmax, double awmax,
                             double rmax, double armax)
{
  // cache
  x0_=x0; y0_=y0; z0_=z0; psi0_=psi0;
  x_goal_=dx_goal; y_goal_=0.0;
  u_term_=u_term;
  vmax_u_=vmax_u; vmax_v_=vmax_v;
  amax_t_=amax_t; wmax_=wmax; awmax_=awmax;
  rmax_=rmax; armax_=armax;

  path_.clear(); t_of_s_.clear(); T_total_=0.0;

  // LOS direction (start -> dock)
  double dx = x_goal_ - x0_;
  double dy = y_goal_ - y0_;
  double Ltot = std::hypot(dx,dy); if (Ltot < 1e-9) Ltot=1e-9;
  double cL = dx/Ltot, sL = dy/Ltot;
  double psi_LOS = std::atan2(sL, cL);

  // Clothoid parameters from angular limits at speed u_term
  kR_   = (u_term_ > 1e-6) ? (rmax_ / u_term_) : 0.0;
  sigma_= (u_term_ > 1e-6) ? (armax_ / (u_term_*u_term_)) : 0.0;

  if (std::abs(kR_) < 1e-9 || std::abs(sigma_) < 1e-12) {
    // Degenerate: just go straight
    const double ds = 0.05;
    buildStraight(x0_, y0_, psi_LOS, Ltot, path_, ds);
    parameterizeTimeConstantSpeed(path_, std::max(0.2,u_term_), t_of_s_);
    T_total_ = t_of_s_.empty()? 0.0 : t_of_s_.back();
    planDepth(z0_, wmax_, awmax_);
    planned_ = true;
    return true;
  }

  // Desired heading change: LOS -> 0 (shortest)
  const double dpsi = angleDiff(0.0, psi_LOS);

  // CAC displacement in local frame (start heading=0), numerically (no Fresnel)
  const double ds_geom = 0.02; // fine step for geometry
  CACResult res = cac_displacement(/*kR signed*/ (dpsi>=0? +kR_ : -kR_),
                                   /*sigma signed*/ std::abs(sigma_),
                                   /*dpsi_total*/ std::abs(dpsi),
                                   ds_geom);

  // Rotate CAC displacement to world (start heading psi_LOS)
  double dx_cac_world =  std::cos(psi_LOS)*res.dx - std::sin(psi_LOS)*res.dy;
  double dy_cac_world =  std::sin(psi_LOS)*res.dx + std::cos(psi_LOS)*res.dy;

  // Guard distance and final straight (closed-form)
  double dr = 0.0, Lfinal = 0.0;
  if (std::abs(sL) > 1e-6) {
    dr     = dy_cac_world / sL;
    Lfinal = dr*cL - dx_cac_world;
  } else {
    // LOS almost along +x: keep y=0; set dr from x eqn ensuring non-negative Lfinal
    dr     = std::max(0.0, dx_cac_world / std::max(1e-6, cL));
    Lfinal = std::max(0.0, dr*cL - dx_cac_world);
  }
  dr      = std::max(0.0, dr);
  Lfinal  = std::max(0.0, Lfinal);

  // Pre-blend point
  double x_pre = x_goal_ - dr * cL;
  double y_pre = y_goal_ - dr * sL;

  // Build full path: LOS -> CAC -> final straight
  const double ds = 0.05;

  // Phase 2: LOS
  buildStraight(x0_, y0_, psi_LOS, std::hypot(x_pre-x0_, y_pre-y0_), path_, ds);

  // Phase 3: CAC (start at (x_pre,y_pre,psi_LOS))
  Lc_ = res.Lc; La_ = res.La;
  const double kR_signed    = (dpsi>=0? +kR_            : -kR_);
  const double sigma_signed = (dpsi>=0? +std::abs(sigma_): -std::abs(sigma_));
  buildCAC(x_pre, y_pre, psi_LOS, kR_signed, sigma_signed, Lc_, La_, path_, ds);

  // Phase 4: final straight (yaw=0)
  buildStraight(path_.back().x, path_.back().y, /*psi=*/0.0, Lfinal, path_, ds);
  L_LOS_   = std::hypot(x_pre-x0_, y_pre-y0_);
  L_CAC_   = 2*Lc_ + La_;
  L_final_ = Lfinal;

  // Time map: constant u_term (simple and smooth)
  parameterizeTimeConstantSpeed(path_, std::max(0.2,u_term_), t_of_s_);
  T_total_ = t_of_s_.empty()? 0.0 : t_of_s_.back();

  // Depth
  planDepth(z0_, wmax_, awmax_);
  planned_ = true;
  return true;
}

// ===== EVALUATE =====
void TrajectoryPlanner::evaluate(double t,
                                 double& x, double& y, double& z,
                                 double& xd, double& yd, double& zd,
                                 double& xdd, double& ydd, double& zdd,
                                 double& yaw, double& r, double& ar) const {
  // defaults
  x=x0_; y=y0_; z=z0_;
  xd=yd=zd=0.0; xdd=ydd=zdd=0.0; yaw=psi0_; r=ar=0.0;
  if (!planned_ || path_.size()<2) { evalDepth(t,z,zd,zdd); return; }

  // Depth independent
  evalDepth(t, z, zd, zdd);

  // Clamp time
  if (t <= 0.0) {
    x=path_.front().x; y=path_.front().y; yaw=path_.front().psi;
    r = u_term_ * path_.front().kappa; ar=0.0;
    return;
  }
  if (t >= T_total_) {
    x=path_.back().x; y=path_.back().y; yaw=path_.back().psi;
    r = u_term_ * path_.back().kappa; ar=0.0;
    return;
  }

  // Locate segment (binary search)
  int hi = std::upper_bound(t_of_s_.begin(), t_of_s_.end(), t) - t_of_s_.begin();
  int lo = std::max(0, hi-1);
  hi = std::min(hi, (int)path_.size()-1);

  double t0 = t_of_s_[lo], t1 = t_of_s_[hi];
  double a  = (t1>t0) ? (t - t0)/(t1 - t0) : 0.0;

  const auto& P0 = path_[lo];
  const auto& P1 = path_[hi];

  // Interpolate pose & curvature
  x   = (1-a)*P0.x   + a*P1.x;
  y   = (1-a)*P0.y   + a*P1.y;
  yaw = wrapToPi((1-a)*P0.psi + a*P1.psi);
  double kappa = (1-a)*P0.kappa + a*P1.kappa;

  // Vel / Acc from constant surge model
  const double u = std::max(0.0, u_term_);
  xd  = u * std::cos(yaw);
  yd  = u * std::sin(yaw);
  r   = u * kappa;

  // Approximate curvature derivative wrt time
  double ds = std::max(1e-4, P1.s - P0.s);
  double dkds = (P1.kappa - P0.kappa) / ds;
  double dkdT = dkds * u; // s-dot = u
  ar  = u * dkdT;         // r = u*kappa -> rdot = u*dk/dt

  // Tangential accel ~ 0 under constant surge; lateral accel = u^2*kappa
  double an = u*u*kappa;
  double tx = std::cos(yaw), ty=std::sin(yaw);
  double nx = -std::sin(yaw), ny=std::cos(yaw);
  xdd = /*a_t*/0.0*tx + an*nx;
  ydd = /*a_t*/0.0*ty + an*ny;
}
