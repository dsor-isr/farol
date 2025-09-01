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

def main():
    rospy.init_node("se3_ref_circle")

    # --- params ---
    hz        = rospy.get_param("~hz", 50.0)
    frame_id  = rospy.get_param("~frame_id", "map")
    R         = rospy.get_param("~radius", 10.0)     # meters
    vmax      = rospy.get_param("~speed", 0.3)       # m/s (clamped to 0.3)
    z_const   = rospy.get_param("~z", 0.0)           # keep z constant
    pub_axes  = rospy.get_param("~publish_disable_axis", True)

    vmax = min(float(vmax), 0.3)                     # hard cap at 0.3 m/s
    if R <= 0.0: 
        rospy.logfatal("radius must be > 0")
        return
    w = vmax / R                                     # rad/s; ensures speed = R*w <= 0.3

    pub = rospy.Publisher("/myellow0/docking/trajectory", SE3Ref, queue_size=10)
    rate = rospy.Rate(hz)
    t0 = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec() - t0
        theta = w * t

        # position (starts at (R,0,0) = (10,0,0))
        x = R * math.cos(theta)
        y = R * math.sin(theta)
        z = z_const

        # velocity (world)
        xd = -R * w * math.sin(theta)
        yd =  R * w * math.cos(theta)
        zd = 0.0

        # acceleration (world)
        xdd = -R * (w**2) * math.cos(theta)
        ydd = -R * (w**2) * math.sin(theta)
        zdd = 0.0

        # yaw tangent to path (from velocity direction)
        psi = math.atan2(yd, xd)                     # heading of velocity vector
        # For a circle at constant speed: psidot = w, psiddot = 0
        psidot = w
        psiddot = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, psi)

        msg = SE3Ref()
        msg.header = Header(stamp=now, frame_id=frame_id)
        msg.p   = Point(x, y, z)
        msg.pd  = Vector3(xd, yd, zd)
        msg.pdd = Vector3(xdd, ydd, zdd)
        msg.q   = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.wd  = Vector3(0.0, 0.0, psidot)         # desired body rates
        msg.wdd = Vector3(0.0, 0.0, psiddot)

        # Optional: disable roll/pitch moments, keep Fx,Fy,Fz,Mz enabled
        if pub_axes and hasattr(msg, "disable_axis"):
            msg.disable_axis = [False, False, True, True, True, False]

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
