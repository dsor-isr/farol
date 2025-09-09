#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# ======================= Quintic (minimum-jerk) time-scaling =======================

def quintic_coeffs(L, v0, a0, vT, aT, T):
    """
    Solve for coefficients c of s(t)=Σ c_i t^i with:
      s(0)=0, s'(0)=v0, s''(0)=a0,
      s(T)=L, s'(T)=vT, s''(T)=aT
    """
    c0 = 0.0
    c1 = v0
    c2 = 0.5 * a0
    A = np.array([
        [T**3,     T**4,      T**5],
        [3*T**2,   4*T**3,    5*T**4],
        [6*T,      12*T**2,   20*T**3]
    ], dtype=float)
    b = np.array([
        L - (c0 + c1*T + c2*T**2),
        vT - (c1 + 2*c2*T),
        aT - (2*c2)
    ], dtype=float)
    c3, c4, c5 = np.linalg.solve(A, b)
    return np.array([c0, c1, c2, c3, c4, c5])

def eval_quintic(c, t):
    """Evaluate s, v=ds/dt, a=d2s/dt2, j=d3s/dt3 for polynomial s(t)=Σ c_i t^i."""
    t = np.asarray(t)
    s = c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5
    v = c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4
    a = 2*c[2] + 6*c[3]*t + 12*c[4]*t**2 + 20*c[5]*t**3
    j = 6*c[3] + 24*c[4]*t + 60*c[5]*t**2
    return s, v, a, j

def choose_duration_for_limits(L, v0, a0, vT, aT, vmax, amax, jmax, T_guess):
    """
    Pick the smallest T >= T_guess such that the quintic time-scaling obeys
    |v|<=vmax, |a|<=amax, |j|<=jmax (checked on a dense grid).
    """
    T = max(T_guess, 1e-3)
    for _ in range(60):  # expand if limits are violated
        c = quintic_coeffs(L, v0, a0, vT, aT, T)
        tt = np.linspace(0, T, 1001)
        _, v, a, j = eval_quintic(c, tt)
        if (np.max(np.abs(v)) <= vmax + 1e-9 and
            np.max(np.abs(a)) <= amax + 1e-9 and
            np.max(np.abs(j)) <= jmax + 1e-9):
            return T, c
        T *= 1.1  # lengthen to reduce peaks
    return T, c

# ======================= LOS + S-curve planner =======================

def plan_los_scurve(
    p0, psi0, goal, u_term,
    vmax_body_u, vmax_body_v,   # (vmax_body_v kept for future overlap checks)
    amax_t, jmax_t,             # along-path accel/jerk limits
    rmax, armax, jrmax,         # yaw-rate/accel/jerk limits
    z0, wmax, awmax, jwmax      # depth limits
):
    """
    Plan:
      - Yaw-in: psi0 -> psi_LOS (quintic S-curve, zero rate/acc at ends)
      - LOS translation: straight from p0->goal with final speed u_term (quintic S-curve)
      - Yaw-out: psi_LOS -> 0, scheduled to finish with translation (may overlap)
      - Depth: z0 -> 0 (quintic S-curve starting at t=0)
    Returns a dict with time vector and reference trajectories.
    """
    p0 = np.asarray(p0, dtype=float)
    goal = np.asarray(goal, dtype=float)
    dp = goal - p0
    L = np.linalg.norm(dp[:2])
    d_hat = dp[:2] / max(L, 1e-9)
    psi_los = np.arctan2(d_hat[1], d_hat[0])

    # ---- Yaw-in ----
    dpsi1 = np.arctan2(np.sin(psi_los - psi0), np.cos(psi_los - psi0))
    T1_guess = max(
        2.0*np.sqrt(abs(dpsi1)/max(armax,1e-6)),
        abs(dpsi1)/max(rmax,1e-6)
    )
    T1, c_yaw1 = choose_duration_for_limits(
        L=abs(dpsi1), v0=0.0, a0=0.0, vT=0.0, aT=0.0,
        vmax=rmax, amax=armax, jmax=jrmax, T_guess=T1_guess
    )

    # ---- Straight translation (end speed = u_term) ----
    v0 = 0.0; a0 = 0.0; vT = max(u_term, 0.0); aT = 0.0
    T2_guess = max(
        L / max(1e-6 + 0.5*max(vT, 1e-6), 1e-6),
        np.sqrt(L/max(amax_t,1e-6))
    )
    T2, c_lin = choose_duration_for_limits(
        L=L, v0=v0, a0=a0, vT=vT, aT=aT,
        vmax=vmax_body_u, amax=amax_t, jmax=jmax_t,
        T_guess=T2_guess
    )

    # ---- Yaw-out (finish with translation) ----
    dpsi2 = np.arctan2(np.sin(0.0 - psi_los), np.cos(0.0 - psi_los))
    T3_guess = max(
        2.0*np.sqrt(abs(dpsi2)/max(armax,1e-6)),
        abs(dpsi2)/max(rmax,1e-6)
    )
    T3, c_yaw2 = choose_duration_for_limits(
        L=abs(dpsi2), v0=0.0, a0=0.0, vT=0.0, aT=0.0,
        vmax=rmax, amax=armax, jmax=jrmax, T_guess=T3_guess
    )
    t0_yaw2 = T1 + T2 - T3  # start time so yaw-out ends at arrival

    # ---- Depth ----
    Dz = abs(z0)
    Tz_guess = max(2.0*np.sqrt(Dz/max(awmax,1e-6)), Dz/max(wmax,1e-6))
    Tz, c_z = choose_duration_for_limits(
        L=Dz, v0=0.0, a0=0.0, vT=0.0, aT=0.0,
        vmax=wmax, amax=awmax, jmax=jwmax, T_guess=Tz_guess
    )

    # ---- Evaluate over time ----
    T_total = T1 + T2
    t = np.linspace(0, T_total, 1200)

    # yaw-in
    s1, r1, ar1, _ = eval_quintic(c_yaw1, np.clip(t, 0, T1))
    yaw_in = psi0 + np.sign(dpsi1) * s1
    r_in   = np.sign(dpsi1) * r1
    ar_in  = np.sign(dpsi1) * ar1

    # translation
    s2, v2, a2, _ = eval_quintic(c_lin, np.clip(t - T1, 0, T2))

    # yaw-out
    tau3 = np.clip(t - t0_yaw2, 0, T3)
    s3, r3, ar3, _ = eval_quintic(c_yaw2, tau3)
    yaw_out = np.sign(dpsi2) * s3
    r_out   = np.sign(dpsi2) * r3
    ar_out  = np.sign(dpsi2) * ar3

    psi = yaw_in + yaw_out
    r   = r_in + r_out
    ar  = ar_in + ar_out

    # world-frame straight-line motion
    x = p0[0] + d_hat[0] * s2
    y = p0[1] + d_hat[1] * s2
    xd = d_hat[0] * v2
    yd = d_hat[1] * v2
    xdd = d_hat[0] * a2
    ydd = d_hat[1] * a2

    # depth
    tz = np.clip(t, 0, Tz)
    sz, vz, az, _ = eval_quintic(c_z, tz)
    z = z0 - np.sign(z0) * sz
    zd = -np.sign(z0) * vz
    zdd = -np.sign(z0) * az

    return dict(
        t=t, x=x, y=y, z=z, psi=psi,
        xd=xd, yd=yd, zd=zd,
        xdd=xdd, ydd=ydd, zdd=zdd,
        r=r, ar=ar, v_along=v2, a_along=a2,
        T1=T1, T2=T2, T3=T3, Tz=Tz, psi_los=psi_los
    )

# ======================= Demo / plotting =======================

if __name__ == "__main__":
    # Initial pose and goal (homing point)
    p0 = np.array([20.0, -8.0, -5.0])   # x0, y0, z0 [m]
    psi0 = np.deg2rad(130.0)            # initial yaw [rad]
    goal = np.array([5.0, 0.0, 0.0])    # homing point (dx, 0, 0)

    # Limits / design params
    u_term = 0.6            # desired terminal surge at arrival [m/s]
    vmax_body_u = 1.2       # max surge [m/s]
    vmax_body_v = 0.4       # (reserved for overlap checks)
    amax_t = 0.3            # along-path accel limit [m/s^2]
    jmax_t = 0.8            # along-path jerk cap [m/s^3]

    rmax   = np.deg2rad(30.0)    # yaw rate limit [rad/s]
    armax  = np.deg2rad(60.0)    # yaw accel limit [rad/s^2]
    jrmax  = np.deg2rad(500.0)   # yaw jerk cap [rad/s^3]

    z0 = p0[2]
    wmax  = 0.5      # vertical speed limit [m/s]
    awmax = 0.7      # vertical accel limit [m/s^2]
    jwmax = 2.0      # vertical jerk cap [m/s^3]

    traj = plan_los_scurve(
        p0=p0, psi0=psi0, goal=goal, u_term=u_term,
        vmax_body_u=vmax_body_u, vmax_body_v=vmax_body_v,
        amax_t=amax_t, jmax_t=jmax_t,
        rmax=rmax, armax=armax, jrmax=jrmax,
        z0=z0, wmax=wmax, awmax=awmax, jwmax=jwmax
    )

    t = traj["t"]

    # XY path
    plt.figure()
    plt.plot(traj["x"], traj["y"])
    plt.axis('equal')
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.title("Planar path (LOS straight)")

    # Positions vs time
    plt.figure()
    plt.plot(t, traj["x"], label="x")
    plt.plot(t, traj["y"], label="y")
    plt.plot(t, traj["z"], label="z")
    plt.xlabel("time [s]"); plt.ylabel("position [m]")
    plt.title("Positions vs time"); plt.legend()

    # Velocities vs time
    plt.figure()
    plt.plot(t, traj["v_along"], label="along-path speed")
    plt.plot(t, traj["xd"], label="x_dot")
    plt.plot(t, traj["yd"], label="y_dot")
    plt.plot(t, traj["zd"], label="z_dot")
    plt.xlabel("time [s]"); plt.ylabel("velocity [SI]")
    plt.title("Velocities vs time"); plt.legend()

    # Accelerations vs time
    plt.figure()
    plt.plot(t, traj["a_along"], label="along-path accel")
    plt.plot(t, traj["xdd"], label="x_ddot")
    plt.plot(t, traj["ydd"], label="y_ddot")
    plt.plot(t, traj["zdd"], label="z_ddot")
    plt.xlabel("time [s]"); plt.ylabel("acceleration [SI]")
    plt.title("Accelerations vs time"); plt.legend()

    # Yaw vs time
    plt.figure()
    plt.plot(t, np.rad2deg(traj["psi"]), label="yaw [deg]")
    plt.plot(t, np.rad2deg(traj["r"]), label="yaw rate [deg/s]")
    plt.plot(t, np.rad2deg(traj["ar"]), label="yaw accel [deg/s^2]")
    plt.xlabel("time [s]"); plt.ylabel("angles / rates")
    plt.title("Yaw profile"); plt.legend()

    print({
        "T_yaw_in": traj["T1"],
        "T_translate": traj["T2"],
        "T_yaw_out": traj["T3"],
        "T_depth": traj["Tz"],
        "psi_LOS_deg": float(np.rad2deg(traj["psi_los"]))
    })

    plt.show()
