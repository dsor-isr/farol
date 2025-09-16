#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3, Quaternion
from farol_docking.msg import SE3Ref

try:
    from tf.transformations import quaternion_from_euler
except Exception:
    def quaternion_from_euler(roll, pitch, yaw):
        cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
        cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
        cy = math.cos(yaw*0.5);   sy = math.sin(yaw*0.5)
        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        y = cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*sp*cy
        return (x, y, z, w)

# ------------------ Robust jerk-limited rest-to-rest S-curve ------------------ #
class SCurve1D:
    """
    Time-optimal, jerk-limited, rest-to-rest motion over distance L >= 0.
    Limits: |v|<=vmax, |a|<=amax, |j|<=jmax. Symmetric 7-phase profile.
    No discontinuities in p, v, or a. Numerically robust across regimes.
    """
    def __init__(self, L, vmax, amax, jmax):
        self.L = float(abs(L))
        self.vmax = float(vmax)
        self.amax = max(1e-9, float(amax))
        self.jmax = max(1e-9, float(jmax))

        if self.L == 0.0:
            self.ta = self.tc = self.tv = self.T = 0.0
            return

        a = self.amax; j = self.jmax
        ta = a / j  # time to ramp 0->amax with jerk j

        # distance for one accel half with const-acc duration tc:
        # s_half(tc) = a*(ta^2 + 1.5*ta*tc + 0.5*tc^2)
        def s_half(tc):
            return a*(ta*ta + 1.5*ta*tc + 0.5*tc*tc)

        # Can we reach vmax (maybe with const-acc)?
        tc_vmax = max(0.0, (self.vmax / a) - ta)
        s_to_vmax = 2.0 * s_half(tc_vmax)

        if self.L >= s_to_vmax - 1e-12:
            # Regime 3: reach vmax and cruise
            self.ta = ta
            self.tc = tc_vmax
            self.tv = (self.L - s_to_vmax) / self.vmax
            self.T  = 2*(2*self.ta + self.tc) + self.tv
            return

        # Regime 1/2: no cruise. Solve L/2 = s_half(tc) for tc >= 0
        A = 0.5*a
        B = 1.5*a*ta
        C = a*ta*ta - (self.L/2.0)
        disc = max(0.0, B*B - 4*A*C)
        tc = (-B + math.sqrt(disc)) / (2*A) if A > 0 else 0.0
        tc = max(0.0, tc)

        # Peak velocity in this regime (should be <= vmax). If not, clamp and add cruise.
        v_peak = a*(ta + tc)
        if v_peak > self.vmax + 1e-9:
            tc = max(0.0, (self.vmax/a) - ta)
            s_needed = 2.0 * s_half(tc)
            self.ta = ta
            self.tc = tc
            self.tv = max(0.0, (self.L - s_needed)/self.vmax)
            self.T  = 2*(2*self.ta + self.tc) + self.tv
            return

        self.ta = ta
        self.tc = tc
        self.tv = 0.0
        self.T  = 2*(2*self.ta + self.tc)

    # --- snap helper to avoid tiny phase-crossing steps ---
    def _snap_time(self, t):
        if self.T == 0.0: return 0.0
        ta, tc, tv = self.ta, self.tc, self.tv
        t1 = ta; t2 = t1 + tc; t3 = t2 + ta
        t4 = t3 + tv; t5 = t4 + ta; t6 = t5 + tc; t7 = t6 + ta
        eps = max(1e-9, 1e-9 * self.T)
        for tb in (0.0, t1, t2, t3, t4, t5, t6, t7):
            if abs(t - tb) <= eps:
                return tb
        if t < 0.0: return 0.0
        if t > self.T: return self.T
        return t

    def sample(self, t):
        """Return (s, v, a, j) for t in [0, T]."""
        if self.T == 0.0:
            return (0.0, 0.0, 0.0, 0.0)
        t = self._snap_time(t)
        if t <= 0.0: return (0.0, 0.0, 0.0, 0.0)
        if t >= self.T: return (self.L, 0.0, 0.0, 0.0)

        ta, tc, tv = self.ta, self.tc, self.tv
        amax, j = self.amax, self.jmax

        # Phase boundaries
        t1 = ta
        t2 = t1 + tc
        t3 = t2 + ta
        t4 = t3 + tv
        t5 = t4 + ta
        t6 = t5 + tc
        t7 = t6 + ta  # == T

        # A1: 0..t1 (j=+j)
        if t <= t1:
            a = j*t
            v = 0.5*j*t*t
            s = (1.0/6.0)*j*t**3
            return (s, v, a, j)

        # values at t1
        s1 = (1.0/6.0)*j*ta**3
        v1 = 0.5*j*ta**2

        # A2: t1..t2 (a=amax)
        if t <= t2:
            dt = t - t1
            a = amax
            v = v1 + amax*dt
            s = s1 + v1*dt + 0.5*amax*dt*dt
            return (s, v, a, 0.0)

        # values at t2
        s2 = s1 + v1*tc + 0.5*amax*tc**2
        v2 = v1 + amax*tc

        # A3: t2..t3 (j=-j)
        if t <= t3:
            dt = t - t2
            a = amax - j*dt
            v = v2 + amax*dt - 0.5*j*dt*dt
            s = s2 + v2*dt + 0.5*amax*dt*dt - (1.0/6.0)*j*dt**3
            return (s, v, a, -j)

        # Cruise: t3..t4
        s3 = s2 + v2*ta + 0.5*amax*ta*ta - (1.0/6.0)*j*ta**3
        v3 = v2 + amax*ta - 0.5*j*ta*ta
        if t <= t4:
            dt = t - t3
            a = 0.0
            v = v3
            s = s3 + v3*dt
            return (s, v, a, 0.0)

        # Decel (mirror), keep absolute offsets so there is no snap
        p4 = s3 + v3*tv     # position at start of decel
        v4 = v3

        # D1: t4..t5 (j=-j)
        if t <= t5:
            dt = t - t4
            a = -j*dt
            v = v4 - 0.5*j*dt*dt
            s = p4 + v4*dt - (1.0/6.0)*j*dt**3
            return (s, v, a, -j)

        # values at t5
        p5 = p4 + v4*ta - (1.0/6.0)*j*ta**3
        v5 = v4 - 0.5*j*ta**2

        # D2: t5..t6 (a=-amax)
        if t <= t6:
            dt = t - t5
            a = -amax
            v = v5 - amax*dt
            s = p5 + v5*dt - 0.5*amax*dt*dt
            return (s, v, a, 0.0)

        # values at t6
        p6 = p5 + v5*tc - 0.5*amax*tc**2
        v6 = v5 - amax*tc

        # D3: t6..t7 (j=+j)
        dt = t - t6
        a = -amax + j*dt
        v = v6 - amax*dt + 0.5*j*dt*dt
        s = p6 + v6*dt - 0.5*amax*dt*dt + (1.0/6.0)*j*dt**3
        return (s, v, a, j)

# ------------------ Square trajectory publisher ------------------ #
def main():
    rospy.init_node("se3_ref_square_scurve")

    # params
    hz        = rospy.get_param("~hz", 10.0)
    frame_id  = rospy.get_param("~frame_id", "map")
    Lside     = rospy.get_param("~side_length", 2)   # meters
    vmax      = rospy.get_param("~vmax", 0.10)         # m/s
    amax      = rospy.get_param("~amax", 0.10)         # m/s^2
    jmax      = rospy.get_param("~jmax", 0.10)         # m/s^3
    z_const   = rospy.get_param("~z", 0.0)
    pub_axes  = rospy.get_param("~publish_disable_axis", True)

    # safety caps
    vmax = min(float(vmax), 2.0)
    amax = max(1e-6, float(amax))
    jmax = max(1e-6, float(jmax))

    pub = rospy.Publisher("/myellow0/docking/trajectory", SE3Ref, queue_size=10)
    rate = rospy.Rate(hz)

    # square corners centered at origin (clockwise)
    h = 0.5 * Lside
    corners = [(-h, -h), ( h, -h), ( h,  h), (-h,  h)]
    nseg = len(corners)

    # precompute segment planners
    def build_segments(vmax_, amax_, jmax_):
        segs_local = []
        for i in range(nseg):
            x0, y0 = corners[i]
            x1, y1 = corners[(i+1) % nseg]
            dx, dy = x1 - x0, y1 - y0
            L = math.hypot(dx, dy)
            planner = SCurve1D(L, vmax_, amax_, jmax_)
            ux, uy = (dx / L if L > 0 else 0.0, dy / L if L > 0 else 0.0)
            segs_local.append({"p0": (x0, y0), "dir": (ux, uy), "L": L, "plan": planner})
        return segs_local

    segs = build_segments(vmax, amax, jmax)
    for i, s in enumerate(segs):
        rospy.loginfo(f"[edge {i}] T={s['plan'].T:.3f}s  (ta={s['plan'].ta:.3f}, tc={s['plan'].tc:.3f}, tv={s['plan'].tv:.3f})")

    seg_idx = 0
    seg_t0  = rospy.Time.now().to_sec()

    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)  # yaw = 0

    while not rospy.is_shutdown():
        now  = rospy.Time.now()
        tnow = now.to_sec()

        seg  = segs[seg_idx]
        plan = seg["plan"]

        # raw time on current segment (no hard clamp; carryover handles overshoot)
        tseg = tnow - seg_t0
        if tseg < 0.0: tseg = 0.0

        # sample along the edge (internal snap clamps to [0,T])
        s, v, a, _ = plan.sample(tseg)
        x0, y0 = seg["p0"]
        ux, uy = seg["dir"]

        # world states
        x   = x0 + ux * s
        y   = y0 + uy * s
        z   = z_const
        xd  = ux * v
        yd  = uy * v
        xdd = ux * a
        ydd = uy * a

        msg = SE3Ref()
        msg.header = Header(stamp=now, frame_id=frame_id)
        msg.p   = Point(x-3.5, y, z)
        msg.pd  = Vector3(xd, yd, 0.0)
        msg.pdd = Vector3(xdd, ydd, 0.0)
        msg.q   = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.wd  = Vector3(0.0, 0.0, 0.0)
        msg.wdd = Vector3(0.0, 0.0, 0.0)

        if pub_axes and hasattr(msg, "disable_axis"):
            # Fx,Fy,Fz,Mz enabled; roll/pitch moments disabled (if field exists)
            msg.disable_axis = [False, False, False, True, True, False]

        pub.publish(msg)

        # --- Robust handover: publish corner, then advance and CARRY OVER leftover dt ---
        left = (tnow - seg_t0) - plan.T
        if left >= 0.0:
            seg_idx = (seg_idx + 1) % nseg
            seg_t0  = tnow - left   # carry micro leftover into the next edge

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
