#!/usr/bin/env python3

""" 
Developers: Ravi Regalo-> ravi.regalo@tecnico.ulisboa.pt Instituto Superior Tecnico
"""

import rospy
import numpy as np
from farol_msgs.msg import mUSBLFix, mState
from auv_msgs.msg import NavigationStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from math import pi
from tf.transformations import euler_from_quaternion 




def euler_to_rot(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,             cp*cr]
    ])

def rot_to_euler(R):
    if abs(R[2, 0]) < 1.0:
        pitch = -np.arcsin(R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:  # Gimbal lock
        pitch = np.pi/2 if R[2, 0] <= -1.0 else -np.pi/2
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return np.array([roll, pitch, yaw])

def compute_rbe(observer, target):
    dp = target[:3] - observer[:3]
    R = euler_to_rot(*observer[3:])
    rel = R.T @ dp  
    rng = np.linalg.norm(rel)
    bearing = np.arctan2(rel[1], rel[0])         # Y over X = East over North
    elev = np.arcsin(-rel[2] / rng) if rng > 1e-6 else 0.0  # Negative Z is down
    return np.array([rng, bearing, -elev])

def pose_to_matrix(pose):
    T = np.eye(4)
    T[:3, :3] = euler_to_rot(*pose[3:])
    T[:3, 3] = pose[:3]
    return T

def matrix_to_pose(T):
    return np.concatenate((T[:3, 3], rot_to_euler(T[:3, :3])))

def auv_in_dock_frame(auv_pose, dock_pose):
    T_rel = np.linalg.inv(pose_to_matrix(dock_pose)) @ pose_to_matrix(auv_pose)
    return matrix_to_pose(T_rel)

ORIGIN = [4290794.43, 491936.56]

def rotation_matrix_yaw(yaw):
    """
    Compute 3D rotation matrix for a given yaw (rotation around z-axis).
    """
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])

def compute_range_bearing_elevation(P):
    """
    Convert a position vector to range, bearing, and elevation.
    P is a 3D vector (x, y, z) in a local frame.
    Returns (range, bearing, elevation).
    """
    range_val = np.linalg.norm(P)  # Distance in 3D
    bearing = np.arctan2(P[1], P[0])  # Azimuthal angle in XY plane
    elevation = np.arctan2(P[2], np.sqrt(P[0]**2 + P[1]**2))  # Elevation angle
    return range_val, bearing, elevation


def enu_to_ned_yaw(yaw_enu):
    # Convert ENU yaw to NED yaw
    yaw_ned = np.pi / 2 - yaw_enu
    
    # Wrap yaw to the range [-pi, pi]
    yaw_ned = (yaw_ned + np.pi) % (2 * np.pi) - np.pi
    return yaw_ned

def wrapToPi(angle):
    return (angle + pi) % (2 * pi) - pi

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def orientation_to_yaw(orientation):
    """
    Convert a ROS Pose orientation (geometry_msgs/Quaternion) to a yaw angle in radians.
    :param orientation: A geometry_msgs/Quaternion object with fields x, y, z, w
    :return: Yaw angle in radians, wrapped to [-pi, pi]
    """
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    
    # Compute yaw (rotation around Z axis)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    # Wrap yaw to [-pi, pi]
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
    return enu_to_ned_yaw(yaw)




class DockingHelperNode():
    def __init__(self):
        rospy.init_node("docking_helper_node")
        self.state = None
        self.state_dock = None
        self.position_noise = 0.5
        self.sim_usbl = False
        self.outliers = False
        self.noise = False
        self.initializeSubscribers()
        self.initializePublishers()
        self.loadParams()
        self.cnt = 0
        self.range_outlier_deviation = 30
        self.bearing_outlier_deviation = 100*pi/180
        self.elevation_outlier_deviation = 80*pi/180
        self.range_noise_stddev = 0.05
        self.bearing_noise_stddev = 0.5/180*pi
        self.elevation_noise_stddev = 0.5/180*pi
        

    def initializeSubscribers(self):
        rospy.Subscriber(rospy.get_param('~' + "topics/subscribers/gazebo"), ModelStates, self.gazebo_callback)
    
    def loadParams(self):
        self.sim_usbl = rospy.get_param('~sim_usbl')
        self.noise = rospy.get_param('~noise')
        self.outliers = rospy.get_param('~outliers')

    def initializePublishers(self):
        self.pub_usbl_fix = rospy.Publisher(rospy.get_param('~' + "topics/publishers/usbl_fix"), mUSBLFix, queue_size=5)
        self.pub_usbl_acomms = rospy.Publisher(rospy.get_param('~' + "topics/publishers/usbl_acomms"), mUSBLFix, queue_size=5)
        self.pub_dock_pose = rospy.Publisher(rospy.get_param('~' + "topics/publishers/dock_pose"), mState, queue_size=5)
        self.pub_docking_gt = rospy.Publisher(rospy.get_param('~' + "topics/publishers/docking_state_gt"), NavigationStatus, queue_size=5)

    def gazebo_callback(self, msg):
        try:
            (roll, pitch, yaw) = euler_from_quaternion([msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w])
            # convert position from ENU to NED
            (x,y,z) = (msg.pose[1].position.y, msg.pose[1].position.x, -msg.pose[1].position.z)
            # convert orientation from ENU to  NED
            (roll, pitch, yaw) =(roll, -pitch, wrap_to_pi(-yaw+pi/2))
            self.state = np.array([x, y, z, roll, pitch, yaw])
            
            (roll, pitch, yaw) = euler_from_quaternion([msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w])
            # convert position from ENU to NED
            (x,y,z) = (msg.pose[2].position.y, msg.pose[2].position.x, -msg.pose[2].position.z)
            # convert orientation from ENU to  NED
            (roll, pitch, yaw) =(roll, -pitch, wrap_to_pi(-yaw+pi/2))
            self.state_dock = np.array([x, y, z, roll, pitch, yaw])
        except:
            pass
    
    def run(self):
        """
        Compute the USBL measurements in both AUV and dock frames.
        `self.state`: [x, y, z, yaw] - position and yaw of the AUV
        `self.state_dock`: [x, y, z, yaw] - position and yaw of the dock
        `position_noise`: noise level for the measurement
        Returns: array of range, bearing, and elevation for both AUV and dock frames.
        """
        if self.state is None or self.state_dock is None:
            return
        
        if self.sim_usbl:
            self.cnt+=1
            if self.cnt>=20:
                self.cnt = 0
                # Calculate the distance between AUV and dock
                distance = np.linalg.norm(self.state[:3] - self.state_dock[:3])

                # Position difference in the inertial frame
                P_DB = self.state[:3] - self.state_dock[:3]

                # Rotate into the AUV body frame
                USBL_B = rotation_matrix_yaw(-self.state[3]) @ (-P_DB)
            #    USBL_B += self.position_noise * (1 + np.tanh(0.4 * distance - 3)) * np.random.randn(USBL_B.shape())

                # Rotate into the dock frame
                USBL_D = rotation_matrix_yaw(-self.state_dock[3]) @ P_DB
                # USBL_D += self.position_noise * (1 + np.tanh(0.4 * distance - 3)) * np.random.randn(USBL_D.shape())

                # print(P_DB, USBL_B,USBL_D)
                # self.pub_usbl_debug.publish(Point(USBL_B[0],USBL_B[1],USBL_B[2]))
                # self.pub_usbl_debug2.publish(Point(USBL_D[0],USBL_D[1],USBL_D[2]))

                # Convert to range-bearing-elevation
                range_bearing_elevation_B = compute_rbe(self.state, self.state_dock)
                range_bearing_elevation_D = compute_rbe(self.state_dock, self.state)
                
                msg1 = mUSBLFix()
                msg1.range = range_bearing_elevation_B[0]
                msg1.header.frame_id = "usbl"
                msg1.type = mUSBLFix.RANGE_ONLY   
                msg1.ahrs_roll = self.state[3]     
                msg1.ahrs_pitch = self.state[4]     
                msg1.ahrs_yaw = self.state[5]     
                
                msg2 = mUSBLFix()
                msg2.bearing_body = range_bearing_elevation_B[1]
                msg2.elevation_body = range_bearing_elevation_B[2]
                msg2.type = mUSBLFix.AZIMUTH_ONLY
                msg2.header.frame_id = "usbl"
                msg2.ahrs_roll = self.state[3]     
                msg2.ahrs_pitch = self.state[4]     
                msg2.ahrs_yaw = self.state[5]  
                
                msg3 = mUSBLFix()
                msg3.bearing_body = range_bearing_elevation_D[1]
                msg3.elevation_body = range_bearing_elevation_D[2]
                msg3.type = mUSBLFix.AZIMUTH_ONLY
                msg3.header.frame_id = "dock"
                msg3.ahrs_roll = self.state_dock[3]     
                msg3.ahrs_pitch = self.state_dock[4]     
                msg3.ahrs_yaw = self.state_dock[5]  
                
                msg4 = mUSBLFix()
                msg4.range = range_bearing_elevation_B[0]
                msg4.header.frame_id = "dock"
                msg4.type = mUSBLFix.RANGE_ONLY
                msg4.ahrs_roll = self.state_dock[3]     
                msg4.ahrs_pitch = self.state_dock[4]     
                msg4.ahrs_yaw = self.state_dock[5]  
                
                if self.outliers:
                    if np.random.rand() < 0.1: # 10% os messages are outliers 
                        print("----------------------------------------------------***\n\n*************************************************")
                        msg1.range = msg1.range  + np.random.uniform(-self.range_outlier_deviation, self.range_outlier_deviation)
                        msg4.range = msg4.range  + np.random.uniform(-self.range_outlier_deviation, self.range_outlier_deviation)
                        msg2.bearing = msg2.bearing  + np.random.uniform(-self.bearing_outlier_deviation, self.bearing_outlier_deviation)
                        msg2.elevation = msg2.elevation  + np.random.uniform(-self.elevation_outlier_deviation, self.elevation_outlier_deviation)
                    if np.random.rand() < 0.1: # 10% os messages are outliers 
                        msg3.bearing = msg3.bearing  + np.random.uniform(-self.bearing_outlier_deviation, self.bearing_outlier_deviation)
                        msg3.elevation = msg3.elevation  + np.random.uniform(-self.elevation_outlier_deviation, self.elevation_outlier_deviation)
                if self.noise:
                    msg1.range = msg1.range  + np.random.normal(0, self.range_noise_stddev)
                    msg4.range = msg4.range  + np.random.normal(0, self.range_noise_stddev)
                    msg2.bearing = msg2.bearing  + np.random.normal(0, self.bearing_noise_stddev)
                    msg2.elevation = msg2.elevation  + np.random.normal(0, self.elevation_noise_stddev)
                    msg3.bearing = msg3.bearing  + np.random.normal(0, self.bearing_noise_stddev)
                    msg3.elevation = msg3.elevation  + np.random.normal(0, self.elevation_noise_stddev)
                    
                self.pub_usbl_fix.publish(msg1)
                self.pub_usbl_fix.publish(msg2)
                self.pub_usbl_acomms.publish(msg3)
                self.pub_usbl_acomms.publish(msg4)
        
        msg = mState()
        msg.X = self.state_dock[1] + ORIGIN[1]
        msg.Y = self.state_dock[0] + ORIGIN[0]
        msg.Yaw = self.state_dock[3]*180/np.pi 
        msg.Z = self.state_dock[2]
        self.pub_dock_pose.publish(msg)
        
        rel_pose = auv_in_dock_frame(self.state, self.state_dock)
        msg = NavigationStatus()
        msg.local_position.x = rel_pose[0]
        msg.local_position.y = rel_pose[1]
        msg.local_position.z = rel_pose[2]
        msg.local_attitude.roll = rel_pose[3]
        msg.local_attitude.pitch = rel_pose[4]
        msg.local_attitude.yaw = rel_pose[5]
        self.pub_docking_gt.publish(msg)


if __name__ == '__main__':
    try:
        docking_helper_node = DockingHelperNode()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            docking_helper_node.run()
            r.sleep()
    except rospy.ROSInterruptException:
            pass