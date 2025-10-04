#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fish_profile_driver.py
ROS1 node to add vertical & attitude excitation on top of 2D path following.

Updates per request:
- Pitch < 0 while DESCENT, Pitch > 0 while ASCENT.
- Publish pitch_ref and roll_ref in DEGREES (depth_ref remains meters).
- Roll is a fixed bank (±roll_bank_deg) when |yaw_rate| > yaw_rate_thresh;
  otherwise roll = 0. Roll sign follows yaw-rate sign.

Topics are std_msgs/Float64 for refs, std_msgs/Empty to trigger ascent/descent.
"""

import math
import rospy
from std_msgs.msg import Float64, Empty
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from dsor_paths.msg import PathData

# ---------------------------- CONFIG (defaults) ----------------------------
DEFAULTS = {
    # Topic names (override via ROS params if desired)
    "yaw_rate_topic":    "/bluerov_heavy0/PathData",     # supports Imu/TwistStamped/Vector3Stamped/Odometry/Float64
    "depth_ref_topic":   "/bluerov_heavy0/ref/depth",  # meters
    "pitch_ref_topic":   "/bluerov_heavy0/ref/pitch",
    "roll_ref_topic":    "/bluerov_heavy0/ref/roll",
    "descent_cmd_topic": "/descent",  # std_msgs/Empty
    "ascent_cmd_topic":  "/ascent",   # std_msgs/Empty

    # Depth limits (meters), positive-down convention assumed
    "d_min":  0.5,
    "d_max":  2.5,
    "depth_slew_rate": 0.1,   # m/s ramp
    "depth_eps": 0.2,         # m snap tolerance

    # Pitch behavior (DEGREES; converted internally to rad)
    "pitch_ref_mag_deg": 15.0,     # magnitude while diving/climbing
    "pitch_limit_deg":   20.0,     # safety clamp

    # Banking (roll) behavior (DEGREES)
    "roll_bank_deg":     20.0,     # fixed bank magnitude when turning
    "roll_limit_deg":    30.0,     # safety clamp
    "yaw_rate_thresh":   0.035,     # rad/s threshold (abs) to apply bank
    "yaw_lpf_tau":       0.25,     # s, low-pass filter on yaw-rate; 0 to disable

    # Publishing rate
    "rate_hz": 20.0,

    # Initial depth reference (m)
    "initial_depth_ref": 0.0,
}
# -------------------------------------------------------------------------


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def move_towards(current, target, max_step):
    if max_step <= 0:
        return target
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


class FishProfileNode:
    HOLD, DESCENT, ASCENT = 0, 1, 2

    def __init__(self):
        # Params
        gp = lambda k: rospy.get_param("~" + k, DEFAULTS[k])

        self.depth_ref_topic   = gp("depth_ref_topic")
        self.pitch_ref_topic   = gp("pitch_ref_topic")
        self.roll_ref_topic    = gp("roll_ref_topic")
        self.yaw_rate_topic    = gp("yaw_rate_topic")
        self.descent_cmd_topic = gp("descent_cmd_topic")
        self.ascent_cmd_topic  = gp("ascent_cmd_topic")

        self.d_min = float(gp("d_min"))
        self.d_max = float(gp("d_max"))
        self.depth_slew_rate = float(gp("depth_slew_rate"))
        self.depth_eps = float(gp("depth_eps"))

        self.pitch_ref_mag_deg = float(gp("pitch_ref_mag_deg"))
        self.pitch_limit_deg   = float(gp("pitch_limit_deg"))

        self.roll_bank_deg   = float(gp("roll_bank_deg"))
        self.roll_limit_deg  = float(gp("roll_limit_deg"))
        self.yaw_rate_thresh = float(gp("yaw_rate_thresh"))
        self.yaw_lpf_tau     = float(gp("yaw_lpf_tau"))

        self.rate_hz = float(gp("rate_hz"))
        self.dt = 1.0 / max(1.0, self.rate_hz)

        # State
        self.mode = self.HOLD
        self.depth_ref = float(gp("initial_depth_ref"))
        self.depth_target = self.depth_ref
        self.yaw_rate_raw = 0.0
        self.yaw_rate_filt = 0.0

        # Publishers (latch refs)
        self.pub_depth = rospy.Publisher(self.depth_ref_topic, Float64, queue_size=1, latch=True)
        self.pub_pitch = rospy.Publisher(self.pitch_ref_topic, Float64, queue_size=1, latch=True)
        self.pub_roll  = rospy.Publisher(self.roll_ref_topic,  Float64, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber(self.descent_cmd_topic, Empty, self.on_descent)
        rospy.Subscriber(self.ascent_cmd_topic,  Empty, self.on_ascent)
        rospy.Subscriber(self.yaw_rate_topic, rospy.AnyMsg, self._generic_yawrate_demux)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update_loop)

        rospy.loginfo("fish_profile_driver: ready")
        rospy.loginfo("DESCENT cmd: rostopic pub %s std_msgs/Empty -1", self.descent_cmd_topic)
        rospy.loginfo("ASCENT  cmd: rostopic pub %s std_msgs/Empty -1", self.ascent_cmd_topic)

        # Initial publish (pitch/roll in DEGREES)
        self._publish_refs(pitch_deg=0.0, roll_deg=0.0)

    # --------------------------- Command handlers ---------------------------
    def on_descent(self, _):
        self.mode = self.DESCENT
        self.depth_target = self.d_max
        rospy.loginfo("DESCENT: target depth -> %.3f m, pitch -> -%.1f deg",
                      self.depth_target, self.pitch_ref_mag_deg)

    def on_ascent(self, _):
        self.mode = self.ASCENT
        self.depth_target = self.d_min
        rospy.loginfo("ASCENT: target depth -> %.3f m, pitch -> +%.1f deg",
                      self.depth_target, self.pitch_ref_mag_deg)
    # -----------------------------------------------------------------------

    # ------------------------ Yaw-rate subscription ------------------------
    def _generic_yawrate_demux(self, anymsg):
        """
        Accepts several msg types on the same topic:
          - sensor_msgs/Imu               -> angular_velocity.z
          - geometry_msgs/TwistStamped    -> twist.angular.z
          - geometry_msgs/Vector3Stamped  -> vector.z
          - nav_msgs/Odometry             -> twist.twist.angular.z
          - std_msgs/Float64              -> data (assumed rad/s)
        """
        try:
            m = PathData(); m.deserialize(anymsg._buff)
            self._set_yaw_rate(m.curvature); return
        except Exception: pass

        rospy.logwarn_throttle(5.0, "fish_profile_driver: Unsupported msg on %s", self.yaw_rate_topic)

    def _set_yaw_rate(self, wz):
        self.yaw_rate_raw = float(wz)
    #     if self.yaw_lpf_tau > 0.0:
    #         alpha = self.dt / (self.yaw_lpf_tau + self.dt)
    #         self.yaw_rate_filt += alpha * (self.yaw_rate_raw - self.yaw_rate_filt)
    #     else:
    #         self.yaw_rate_filt = self.yaw_rate_raw
    # # -----------------------------------------------------------------------

    # ------------------------------- Control --------------------------------
    def update_loop(self, _evt):
        # Depth ramp toward target (m)
        max_step = self.depth_slew_rate * self.dt
        self.depth_ref = move_towards(self.depth_ref, self.depth_target, max_step)

        # Pitch in DEGREES (neg when descending, pos when ascending)
        if self.mode == self.DESCENT:
            pitch_deg = -self.pitch_ref_mag_deg
            if abs(self.depth_ref - self.depth_target) <= self.depth_eps:
                self.mode = self.HOLD
                pitch_deg = 0.0
        elif self.mode == self.ASCENT:
            pitch_deg = +self.pitch_ref_mag_deg
            if abs(self.depth_ref - self.depth_target) <= self.depth_eps:
                self.mode = self.HOLD
                pitch_deg = 0.0
        else:
            pitch_deg = 0.0

        # Roll bank logic (DEGREES)
        wz = self.yaw_rate_filt  # rad/s
        if abs(wz) > self.yaw_rate_thresh:
            roll_deg = math.copysign(self.roll_bank_deg, wz)  # sign follows yaw rate
        else:
            roll_deg = 0.0

        # Safety clamps (deg)
        pitch_deg = clamp(pitch_deg, -self.pitch_limit_deg, self.pitch_limit_deg)
        roll_deg  = clamp(roll_deg,  -self.roll_limit_deg,  self.roll_limit_deg)

        # Publish (depth in m, pitch/roll in deg)
        self._publish_refs(pitch_deg=pitch_deg, roll_deg=roll_deg)

    def _publish_refs(self, pitch_deg, roll_deg):
        self.pub_depth.publish(Float64(self.depth_ref))      # meters
        if(abs(pitch_deg) >0):
            self.pub_pitch.publish(Float64(pitch_deg))           # degrees
        if(self.yaw_rate_raw >0):
            self.pub_roll.publish(Float64(-20))
        if(self.yaw_rate_raw <0):
            self.pub_roll.publish(Float64(20))             # degrees
    # -----------------------------------------------------------------------


def main():
    rospy.init_node("fish_profile_driver")
    FishProfileNode()
    rospy.spin()


if __name__ == "__main__":
    main()
