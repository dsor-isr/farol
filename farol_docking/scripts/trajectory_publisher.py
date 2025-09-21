#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3, Quaternion
from farol_docking.msg import SE3Ref
import sys

# ---------------- utils ----------------
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
        z = cr*cp*sy - sr*cp*cy
        return (x, y, z, w)

def clamp(x, lo, hi): return max(lo, min(hi, x))

# ------------- jerk-limited S-curve (rest-to-rest, robust) -------------
class SCurve1D:
    """
    Time-optimal jerk-limited rest-to-rest profile over distance L with bounds
    vmax, amax, jmax. 7-phase symmetric profile. No jumps in p,v,a.
    """
    def __init__(self, L, vmax, amax, jmax):
        self.L = float(abs(L))
        self.vmax = float(vmax)
        self.amax = max(1e-9, float(amax))
        self.jmax = max(1e-9, float(jmax))
        if self.L == 0.0:
            self.ta = self.tc = self.tv = self.T = 0.0; return

        a = self.amax; j = self.jmax
        ta = a / j

        def s_half(tc):  # distance in one accel half (j+, a, j-)
            return a*(ta*ta + 1.5*ta*tc + 0.5*tc*tc)

        tc_vmax = max(0.0, (self.vmax / a) - ta)
        s_to_vmax = 2.0 * s_half(tc_vmax)

        if self.L >= s_to_vmax - 1e-12:
            self.ta, self.tc = ta, tc_vmax
            self.tv = (self.L - s_to_vmax) / self.vmax
            self.T  = 2*(2*self.ta + self.tc) + self.tv
            return

        # No cruise: solve L/2 = s_half(tc)
        A = 0.5*a
        B = 1.5*a*ta
        C = a*ta*ta - (self.L/2.0)
        disc = max(0.0, B*B - 4*A*C)
        tc = (-B + math.sqrt(disc)) / (2*A) if A > 0 else 0.0
        tc = max(0.0, tc)

        v_peak = a*(ta + tc)
        if v_peak > self.vmax + 1e-9:  # clamp + add cruise (rare due to roundoff)
            tc = max(0.0, (self.vmax/a) - ta)
            s_needed = 2.0 * s_half(tc)
            self.ta, self.tc = ta, tc
            self.tv = max(0.0, (self.L - s_needed)/self.vmax)
            self.T  = 2*(2*self.ta + self.tc) + self.tv
            return

        self.ta, self.tc, self.tv = ta, tc, 0.0
        self.T = 2*(2*self.ta + self.tc)

    def _snap_time(self, t):
        if self.T == 0.0: return 0.0
        ta, tc, tv = self.ta, self.tc, self.tv
        t1 = ta; t2 = t1 + tc; t3 = t2 + ta
        t4 = t3 + tv; t5 = t4 + ta; t6 = t5 + tc; t7 = t6 + ta
        eps = max(1e-9, 1e-9 * self.T)
        for tb in (0.0, t1, t2, t3, t4, t5, t6, t7):
            if abs(t - tb) <= eps: return tb
        return clamp(t, 0.0, self.T)

    def sample(self, t):
        """Return (s, v, a) for t in [0, T]."""
        if self.T == 0.0: return (0.0, 0.0, 0.0)
        t = self._snap_time(t)
        if t <= 0.0: return (0.0, 0.0, 0.0)
        if t >= self.T: return (self.L, 0.0, 0.0)

        ta, tc, tv = self.ta, self.tc, self.tv
        amax, j = self.amax, self.jmax
        t1 = ta; t2 = t1 + tc; t3 = t2 + ta
        t4 = t3 + tv; t5 = t4 + ta; t6 = t5 + tc; t7 = t6 + ta

        if t <= t1:
            a = j*t; v = 0.5*j*t*t; s = (1.0/6.0)*j*t**3
            return (s, v, a)

        s1 = (1.0/6.0)*j*ta**3
        v1 = 0.5*j*ta**2
        if t <= t2:
            dt = t - t1
            a = amax
            v = v1 + amax*dt
            s = s1 + v1*dt + 0.5*amax*dt*dt
            return (s, v, a)

        s2 = s1 + v1*tc + 0.5*amax*tc**2
        v2 = v1 + amax*tc
        if t <= t3:
            dt = t - t2
            a = amax - j*dt
            v = v2 + amax*dt - 0.5*j*dt*dt
            s = s2 + v2*dt + 0.5*amax*dt*dt - (1.0/6.0)*j*dt**3
            return (s, v, a)

        s3 = s2 + v2*ta + 0.5*amax*ta*ta - (1.0/6.0)*j*ta**3
        v3 = v2 + amax*ta - 0.5*j*ta*ta
        if t <= t4:
            dt = t - t3
            a = 0.0
            v = v3
            s = s3 + v3*dt
            return (s, v, a)

        p4 = s3 + v3*tv
        v4 = v3
        if t <= t5:
            dt = t - t4
            a = -j*dt
            v = v4 - 0.5*j*dt*dt
            s = p4 + v4*dt - (1.0/6.0)*j*dt**3
            return (s, v, a)

        p5 = p4 + v4*ta - (1.0/6.0)*j*ta**3
        v5 = v4 - 0.5*j*ta**2
        if t <= t6:
            dt = t - t5
            a = -amax
            v = v5 - amax*dt
            s = p5 + v5*dt - 0.5*amax*dt*dt
            return (s, v, a)

        dt = t - t6
        p6 = p5 + v5*tc - 0.5*amax*tc**2
        v6 = v5 - amax*tc
        a = -amax + j*dt
        v = v6 - amax*dt + 0.5*j*dt*dt
        s = p6 + v6*dt - 0.5*amax*dt*dt + (1.0/6.0)*j*dt**3
        return (s, v, a)

# ------------- minimum-jerk half-stroke utilities (C^2) -------------
def mj_poly(sigma):
    # s(σ), v(σ), a(σ) for σ∈[0,1]; s' max = 1.875, a' max = 10/√3, j' max = 60
    s  = 10*sigma**3 - 15*sigma**4 + 6*sigma**5
    v  = 30*sigma**2 - 60*sigma**3 + 30*sigma**4
    a  = 60*sigma    - 180*sigma**2 + 120*sigma**3
    return s, v, a

def mj_time_scale(L, vmax, amax, jmax):
    Tv = (1.875 * L) / vmax if vmax > 0 else 0.0
    Ta = math.sqrt((10.0/math.sqrt(3.0)) * L / amax) if amax > 0 else 0.0
    Tj = (60.0 * L / jmax) ** (1.0/3.0) if jmax > 0 else 0.0
    return max(Tv, Ta, Tj)

# ---------------- trajectories ----------------
class TrajBase:
    def sample(self, t):  # returns (x,y,z, xd,yd,zd, yaw, ydots: (wx,wy,wz), yddots)
        raise NotImplementedError

class CircleTraj(TrajBase):
    def __init__(self, R, speed, z, center=(0.0,0.0), heading_mode="fixed"):
        self.R = max(1e-6, float(R))
        self.v = float(speed)
        self.w = self.v / self.R
        self.cx, self.cy = center
        self.z = z
        self.mode = heading_mode  # "fixed" or "tangent"

    def sample(self, t):
        th = self.w * t
        x = self.cx + self.R*math.cos(th)
        y = self.cy + self.R*math.sin(th)
        z = self.z
        xd = -self.R*self.w*math.sin(th)
        yd =  self.R*self.w*math.cos(th)
        zd = 0.0
        xdd = -self.R*(self.w**2)*math.cos(th)
        ydd = -self.R*(self.w**2)*math.sin(th)
        zdd = 0.0
        if self.mode == "tangent":
            yaw = math.atan2(yd, xd)  # heading of velocity
            wd  = (0.0, 0.0, self.w)  # psidot = w (constant)
            wdd = (0.0, 0.0, 0.0)
        else:
            yaw = 0.0
            wd  = (0.0, 0.0, 0.0)
            wdd = (0.0, 0.0, 0.0)
        return (x,y,z, xd,yd,zd, xdd,ydd,zdd, yaw, wd, wdd)


class SquareMinJerkTraj(TrajBase):
    """
    Square path centered at origin, traversed edge-by-edge with a
    minimum-jerk (C^2) profile per edge. Time-scaling enforces vmax/amax/jmax.
    Stops at each corner (v = 0, a = 0) → no jumps, no slivers.
    """
    def __init__(self, side, vmax, amax, jmax, z, x_off, dwell=0.0):
        self.side = float(side)
        self.h = 0.5 * self.side
        self.z = z
        self.x_off = x_off
        self.dwell = max(0.0, float(dwell))

        # corners (clockwise) and edge directions
        self.corners = [
            (-self.h, -self.h),
            ( self.h, -self.h),
            ( self.h,  self.h),
            (-self.h,  self.h),
        ]
        self.dirs = [
            ( 1.0, 0.0),  # left -> right
            ( 0.0, 1.0),  # bottom -> top
            (-1.0, 0.0),  # right -> left
            ( 0.0,-1.0),  # top -> bottom
        ]

        # One min-jerk “stroke” per edge (length = side)
        L = self.side
        self.Tedge = mj_time_scale(L, vmax, amax, jmax)  # satisfies all limits
        self.Tedge_total = self.Tedge + self.dwell       # travel + dwell at corner
        self.Tloop = 4.0 * self.Tedge_total
        self.L = L  # store for clarity

    def _edge_sample(self, tau):
        """
        Sample along a single edge. tau ∈ [0, Tedge_total).
        During the final dwell window we hold the corner exactly.
        Returns (s, v, a) with s ∈ [0, L].
        """
        if tau >= self.Tedge:     # dwell phase
            return self.L, 0.0, 0.0
        # normalized phase σ
        sigma = clamp(tau / self.Tedge, 0.0, 1.0)
        s_, v_, a_ = mj_poly(sigma)
        s  = self.L * s_
        v  = (self.L / self.Tedge)       * v_
        a  = (self.L / (self.Tedge**2))  * a_
        # belt-and-suspenders clamp
        s = clamp(s, 0.0, self.L)
        return s, v, a

    def sample(self, t):
        if self.Tedge_total <= 0.0:
            # Degenerate: sit at first corner
            x0, y0 = self.corners[0]
            return (x0, y0, self.z, 0,0,0, 0,0,0, 0.0, (0,0,0), (0,0,0))

        # Which edge, and local time within that edge incl. dwell?
        tmod = t % self.Tloop
        k = int(tmod / self.Tedge_total)
        if k >= 4:
            k = 3  # guard
        tau = tmod - k * self.Tedge_total

        # Sample along that edge
        s, v, a = self._edge_sample(tau)

        # Map 1D progress to XY
        x0, y0 = self.corners[k]
        ux, uy = self.dirs[k]
        x = x0 + ux * s
        y = y0 + uy * s
        xd = ux * v
        yd = uy * v
        xdd = ux * a
        ydd = uy * a

        yaw = 0.0
        return (x+self.x_off, y, self.z, xd, yd, 0.0, xdd, ydd, 0.0, yaw, (0,0,0), (0,0,0))


class OscYMinJerkTraj(TrajBase):
    """Hold x; oscillate y between y_c±A using min-jerk halves; limit-aware; C^2 at peaks."""
    def __init__(self, x_const, y_center, amplitude, vmax, amax, jmax, z):
        self.xc = float(x_const)
        self.yc = float(y_center)
        self.A  = abs(float(amplitude))
        self.z  = z
        self.L  = 2.0*self.A
        if self.L > 0:
            self.Thalf = mj_time_scale(self.L, vmax, amax, jmax)
            self.period = 2.0*self.Thalf
        else:
            self.Thalf = 0.0
            self.period = 0.0

    def sample(self, t):
        x = self.xc; z = self.z
        if self.period <= 0.0:
            return (x, self.yc, z, 0,0,0, 0,0,0, 0.0, (0,0,0), (0,0,0))
        tmod = t % self.period
        up = (tmod < self.Thalf)
        tau = tmod if up else (tmod - self.Thalf)
        sigma = clamp(tau / self.Thalf, 0.0, 1.0)
        s_, v_, a_ = mj_poly(sigma)
        s  = self.L * s_
        v  = (self.L / self.Thalf) * v_
        a  = (self.L / (self.Thalf**2)) * a_

        if up:
            y   = (self.yc - self.A) + s
            yd  =  v
            ydd =  a
        else:
            y   = (self.yc + self.A) - s
            yd  = -v
            ydd = -a

        # safety clamp
        y = clamp(y, self.yc - self.A, self.yc + self.A)
        return (x,y,z, 0.0,yd,0.0, 0.0,ydd,0.0, 0.0, (0,0,0), (0,0,0))


class SinusoidTraj(TrajBase):
    """Simple sinusoid on chosen axis (x or y), fixed yaw."""
    def __init__(self, axis, offset_x, offset_y, amplitude, freq_hz, z):
        self.axis = axis.lower()  # 'x' or 'y'
        self.ax = float(amplitude)
        self.omega = 2.0*math.pi*float(freq_hz)
        self.x0 = float(offset_x)
        self.y0 = float(offset_y)
        self.z  = z

    def sample(self, t):
        s = math.sin(self.omega*t)
        c = math.cos(self.omega*t)
        if self.axis == 'x':
            x = self.x0 + self.ax*s
            y = self.y0
            xd = self.ax*self.omega*c
            yd = 0.0
            xdd = -self.ax*(self.omega**2)*s
            ydd = 0.0
        else:
            x = self.x0
            y = self.y0 + self.ax*s
            xd = 0.0
            yd = self.ax*self.omega*c
            xdd = 0.0
            ydd = -self.ax*(self.omega**2)*s
        z = self.z; zd=0.0; zdd=0.0
        yaw = 0.0; wd=(0,0,0); wdd=(0,0,0)
        return (x,y,z, xd,yd,zd, xdd,ydd,zdd, yaw, wd, wdd)

# ---------------- node ----------------
def main(args):
    rospy.init_node("trajectory_plotter")

    # Common params
    if(len(args) > 0):
      mode =args[1]
    else:  
      mode = rospy.get_param("~mode", "~") 
    hz        = float(rospy.get_param("~hz", 10.0))
    frame_id  = rospy.get_param("~frame_id", "map")
    pub_axes  = bool(rospy.get_param("~publish_disable_axis", True))
    z_const   = float(rospy.get_param("~z", -0.5))

    # Build the chosen trajectory
    mode = mode.lower().strip()
    if mode == "circle_fixed" or mode == "circle_tangent":
        R    = float(rospy.get_param("~radius", 0.75))
        vmax = float(rospy.get_param("~speed", 0.1))
        vmax = min(vmax, 1.0)
        traj = CircleTraj(R=R, speed=vmax, z=z_const,
                          center=(float(rospy.get_param("~cx", -2.75)),
                                  float(rospy.get_param("~cy", 0.0))),
                          heading_mode=("tangent" if mode=="circle_tangent" else "fixed"))
    elif mode == "square_scurve":
        Lside = float(rospy.get_param("~side_length", 1.5))
        vmax  = float(rospy.get_param("~vmax", 0.1))
        amax  = float(rospy.get_param("~amax", 0.05))
        jmax  = float(rospy.get_param("~jmax", 0.02))
        xoff  = float(rospy.get_param("~x_off", -2.75))
        traj = SquareMinJerkTraj(Lside, vmax, amax, jmax, z_const, xoff)
    elif mode == "osc_y_minjerk":
        x_const  = float(rospy.get_param("~x", -2.75))
        y_center = float(rospy.get_param("~y_center", 0.0))
        amp      = float(rospy.get_param("~amplitude", 0.75))
        vmax  = float(rospy.get_param("~vmax", 0.10))
        amax  = float(rospy.get_param("~amax", 0.05))
        jmax  = float(rospy.get_param("~jmax", 0.02))
        traj = OscYMinJerkTraj(x_const, y_center, amp, vmax, amax, jmax, z_const)
    elif mode == "sinusoid":
        if(len(args) > 0):
          axis =args[2]
        else:  
          axis   = rospy.get_param("~axis", "y")
        amp    = float(rospy.get_param("~amplitude", 0.75)) 
        freq   = float(rospy.get_param("~freq_hz", 0.05))
        x0     = float(rospy.get_param("~x0", -2.75))
        y0     = float(rospy.get_param("~y0", 0.0))
        traj = SinusoidTraj(axis, x0, y0, amp, freq, z_const)
    else:
        rospy.logwarn("[Error] Please specify a trajectory:\n  - circle_fixed\n  - circle_tangent\n  - square_scurve\n  - osc_y_minjerk\n  - sinusoid")
        return


    pub = rospy.Publisher("/myellow0/docking/trajectory", SE3Ref, queue_size=10)
    rate = rospy.Rate(hz)
    t0 = rospy.Time.now().to_sec()

    rospy.loginfo(f"[trajectory_master] mode={mode}")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t   = now.to_sec() - t0

        x,y,z, xd,yd,zd, xdd,ydd,zdd, yaw, wd, wdd = traj.sample(t)
        qx,qy,qz,qw = quaternion_from_euler(0.0, 0.0, yaw)

        msg = SE3Ref()
        msg.header = Header(stamp=now, frame_id=frame_id)
        msg.p   = Point(x, y, z)
        msg.pd  = Vector3(xd, yd, zd)
        msg.pdd = Vector3(xdd, ydd, zdd)
        msg.q   = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.wd  = Vector3(*wd)
        msg.wdd = Vector3(*wdd)

        if pub_axes and hasattr(msg, "disable_axis"):
            # Enable Fx,Fy,Fz,Mz; disable roll/pitch moments (if field exists)
            msg.disable_axis = [False, False, False, True, True, False]

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
