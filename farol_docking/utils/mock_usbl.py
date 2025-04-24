#!/usr/bin/env python3

""" 
Developers: Ravi Regalo-> ravi.regalo@tecnico.ulisboa.pt Instituto Superior Tecnico
"""

import rospy
import numpy as np
from farol_msgs.msg import mUSBLFix, mState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from math import pi

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




class MockUSBLNode():
    def __init__(self):
        rospy.init_node("usbl_splitter_node")
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
        self.pub_docking_gt = rospy.Publisher(rospy.get_param('~' + "topics/publishers/docking_state_gt"), mState, queue_size=5)
        # self.pub_usbl_debug = rospy.Publisher("mock_usbl_debug", Point, queue_size=5)
        # self.pub_usbl_debug2 = rospy.Publisher("mock_usbl_debug2", Point, queue_size=5)

    def gazebo_callback(self, msg):
        try:
            self.state = np.array([msg.pose[1].position.y, msg.pose[1].position.x, msg.pose[1].position.z, orientation_to_yaw(msg.pose[1].orientation)])
            self.state_dock = np.array([msg.pose[2].position.y, msg.pose[2].position.x, msg.pose[2].position.z, orientation_to_yaw(msg.pose[2].orientation)])
        except:
            pass#rospy.logwarn("Failed to get gazebo position")
    
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
                range_bearing_elevation_B = compute_range_bearing_elevation(USBL_B)
                range_bearing_elevation_D = compute_range_bearing_elevation(USBL_D)
                
                msg1 = mUSBLFix()
                msg1.range = range_bearing_elevation_B[0]
                msg1.header.frame_id = "usbl"
                msg1.type = mUSBLFix.RANGE_ONLY        
                
                msg2 = mUSBLFix()
                msg2.bearing = range_bearing_elevation_B[1]
                msg2.elevation = range_bearing_elevation_B[2]
                msg2.type = mUSBLFix.AZIMUTH_ONLY
                msg2.header.frame_id = "usbl"
                
                msg3 = mUSBLFix()
                msg3.bearing = range_bearing_elevation_D[1]
                msg3.elevation = range_bearing_elevation_D[2]
                msg3.type = mUSBLFix.AZIMUTH_ONLY
                msg3.header.frame_id = "dock"
                
                if self.outliers:
                    if np.random.rand() < 0.1: # 10% os messages are outliers 
                        print("----------------------------------------------------***\n\n*************************************************")
                        msg1.range = msg1.range  + np.random.uniform(-self.range_outlier_deviation, self.range_outlier_deviation)
                        msg2.bearing = msg2.bearing  + np.random.uniform(-self.bearing_outlier_deviation, self.bearing_outlier_deviation)
                        msg2.elevation = msg2.elevation  + np.random.uniform(-self.elevation_outlier_deviation, self.elevation_outlier_deviation)
                    if np.random.rand() < 0.1: # 10% os messages are outliers 
                        msg3.bearing = msg3.bearing  + np.random.uniform(-self.bearing_outlier_deviation, self.bearing_outlier_deviation)
                        msg3.elevation = msg3.elevation  + np.random.uniform(-self.elevation_outlier_deviation, self.elevation_outlier_deviation)
                if self.noise:
                    msg1.range = msg1.range  + np.random.normal(0, self.range_noise_stddev)
                    msg2.bearing = msg2.bearing  + np.random.normal(0, self.bearing_noise_stddev)
                    msg2.elevation = msg2.elevation  + np.random.normal(0, self.elevation_noise_stddev)
                    msg3.bearing = msg3.bearing  + np.random.normal(0, self.bearing_noise_stddev)
                    msg3.elevation = msg3.elevation  + np.random.normal(0, self.elevation_noise_stddev)
                    
                self.pub_usbl_fix.publish(msg1)
                self.pub_usbl_fix.publish(msg2)
                self.pub_usbl_acomms.publish(msg3)
        
        msg = mState()
        msg.X = self.state_dock[1] + ORIGIN[1]
        msg.Y = self.state_dock[0] + ORIGIN[0]
        msg.Yaw = self.state_dock[3]*180/np.pi 
        msg.Z = self.state_dock[2]
        self.pub_dock_pose.publish(msg)
        
        P = rotation_matrix_yaw(-self.state_dock[3]) @ (self.state[:3] - self.state_dock[:3])
        msg = mState()
        msg.X = P[0]
        msg.Y = P[1]
        msg.Yaw = wrapToPi(self.state_dock[3] + self.state[3]) 
        msg.Z = P[2]
        self.pub_docking_gt.publish(msg)


if __name__ == '__main__':
    try:
        mock_usbl_node = MockUSBLNode()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            mock_usbl_node.run()
            r.sleep()
    except rospy.ROSInterruptException:
            pass