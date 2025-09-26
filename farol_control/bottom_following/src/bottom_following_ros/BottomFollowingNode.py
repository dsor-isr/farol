#!/usr/bin/env python

""" 
Developers: Ravi Regalo -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
from bottom_following_algorithms.BottomFollowing import BottomFollowing
from bottom_following_algorithms.OutlierRejection import OutlierRejection
from bottom_following.msg import BottomFollowingDebug
from std_msgs.msg import Float64, Bool, Int8, Empty
from geometry_msgs.msg import Vector3
from rospy_tutorials.msg import Floats
from math import pi, sqrt, acos, atan2
from auv_msgs.msg import NavigationStatus
from dsor_msgs.msg import Measurement
from a50_dvl.msg import DVL
from uuv_sensor_ros_plugins_msgs.msg import DVL as DVL_sim
import numpy as np
np.set_printoptions(precision=3, suppress=True)


def wrap_to_pi(angle):
	return (angle + pi) % (2 * pi) - pi


class BottomFollowingNode():
	def __init__(self):
		# Register node with rosmaster
		rospy.init_node('bottom_following_node')

		# Handy Variables
		self.initialized  = False
		self.alpha = None
		self.h = [None, None, None, None, None]
		self.body_velocity = [None, None, None]
		self.attitude = [None, None, None]
		self.outlier_rejector_altimeter = OutlierRejection(10, 5.0, 1)
		self.outlier_rejector_dvl = OutlierRejection(10, 5.0, 4)
		
		# Controller references and reference timestamps
		self.v_ref=0
		self.v_ref_t = None
		self.heading_ref=0
		self.heading_ref_t = None
		self.d_ref = None
		self.d_ref_t =None
		
		# controller parameters
		self.u_max =0.2
		self.kp =0.1120
		self.ki =0.0064
		
		
		# for inner-outer depth control
		self.depth_int =0
		self.depth_int_max=10
		self.depth_ref=0
		self.depth_time =None
		
		# ROS messages objects for publishing
		self.debug_msg = BottomFollowingDebug()

		"""
		Initializing subscribers, publishers and loading parameters 
		"""
		self.loadParams()
		self.initializeSubscribers()
		self.initializePublishers()
		self.initializeTimer()
		#self.initializeServices()
		self.bottom_follower = None
		self.ouliter_rejector = None
		self.last_time = rospy.Time.now()
		

	"""
	Function to set up parameters; 
	"""
	def loadParams(self):
		self.node_frequency = rospy.get_param('~node_frequency')
		self.alpha =pi/180*rospy.get_param('~alpha', 27)
		self.kp =rospy.get_param('~kp', 0.07)
		self.ki =rospy.get_param('~ki', 0.0025)
			

	"""
	Function to set up subscribers; 
	"""
	def initializeSubscribers(self):
		rospy.loginfo('Initializing Subscribers for BottomFollowingNode')
		# Altimeter sub
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/altimeter', "/bluerov_heavy0/measurement/position"), Measurement, self.altitude_callback)
		# DVL sub
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/dvl_beams', '/bluerov_heavy0/drivers/dvl/data'), DVL, self.dvl_ranges_callback)
		# from simulation only
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/dvl_beams_sim', '/bluerov_heavy0/dvl'), DVL_sim, self.dvl_ranges_sim_callback)
		# body_velocity sub
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/speed', '/bluerov_heavy0/measurement/velocity'), Measurement, self.body_velocity_callback)
		# attitude sub
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/orientation', '/bluerov_heavy0/measurement/orientation'), Measurement, self.attitude_callback)
		# Speed ref sub
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/speed_ref', '/bluerov_heavy0/ref/surge_'), Float64, self.v_ref_callback)
		# Heading ref sub
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/yaw_ref', '/bluerov_heavy0/ref/yaw'), Float64, self.heading_ref_callback)
		# desired distance from terrain sub
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/distance_ref', '/bluerov_heavy0/bottom_following/ref/distance'), Float64, self.d_ref_callback)
		# controller gain sub
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/k_p', '/bluerov_heavy0/bottom_following/Kp'), Float64, self.kp_callback)
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/k_i', '/bluerov_heavy0/bottom_following/Ki'), Float64, self.ki_callback)
		

		# for quick depth controller test
		rospy.Subscriber(rospy.get_param('~/topics/subscribers/depth_ref', '/bluerov_heavy0/bottom_following/ref/depth'), Float64, self.depth_ref_callback)
			
	
	"""
	Function to set up publishers; 
	"""
	def initializePublishers(self):
		rospy.loginfo('Initializing Publishers for BottomFollowingNode')
		self.surge_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/surge','/bluerov_heavy0/ref/surge'), Float64, queue_size=10)
		self.sway_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/sway','/bluerov_heavy0/ref/sway'), Float64, queue_size=10)
		self.heave_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/heave', '/bluerov_heavy0/ref/heave'), Float64, queue_size=10)
		self.roll_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/roll','/bluerov_heavy0/ref/roll'), Float64, queue_size=10)
		self.pitch_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/roll','/bluerov_heavy0/ref/pitch'), Float64, queue_size=10)
		self.yaw_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/roll','/bluerov_heavy0/ref/yaw'), Float64, queue_size=10)
		
		self.flag_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/flag', '/bluerov_heavy0/Flag'), Int8, queue_size=10)
		self.D_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/D', '/bluerov_heavy0/bottom_following/D'), Vector3, queue_size=10)
		self.D_dot_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/D_dot', '/bluerov_heavy0/bottom_following/D_dot'), Vector3, queue_size=10)
		self.debug_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/debug', '/bluerov_heavy0/bottom_following/debug'), BottomFollowingDebug, queue_size=10)
			

	"""
	Function to set up the timer
	"""
	def initializeTimer(self):
		self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


	"""
	Function to shutdown timer;
	"""
	def shutdownTimer(self):
		self.timer.shutdown()


	"""
	All topics callbacks    
	"""
	def altitude_callback(self,msg):
		if "altimeter" in msg.header.frame_id:
				self.h[4] = msg.value[0]# self.outlier_rejector_altimeter.compute(msg.value[0])
		
		# For quick test of depth controller
		if "depth" in msg.header.frame_id:
				self.depth = msg.value[0]
				
	def body_velocity_callback(self, msg):
		self.body_velocity = [msg.value[0], msg.value[1], msg.value[2]] 
			
	def attitude_callback(self, msg):
		# self.attitude = [wrap_to_pi( msg.value[0]/180*pi), wrap_to_pi( msg.value[1]/180*pi), wrap_to_pi( msg.value[2]/180*pi) ]
		self.attitude = [msg.value[0], msg.value[1], msg.value[2] ]
		# rospy.loginfo(self.attitude)
	
	# 
	def dvl_ranges_callback(self, msg):
		new_measurement = np.array([[msg.beams[0].distance ,msg.beams[1].distance ,msg.beams[2].distance ,msg.beams[3].distance ]]).T
		#new_measurement = self.outlier_rejector_dvl.compute(new_measurement)
		self.h[0] = new_measurement[0,0]
		self.h[1] = new_measurement[1,0]
		self.h[2] = new_measurement[2,0] 
		self.h[3] = new_measurement[3,0] 
	
	# This is just because in simulation the beams are diferent bruhh
	def dvl_ranges_sim_callback(self, msg):
		new_measurement = np.array([msg.beams[0].range ,msg.beams[1].range ,msg.beams[2].range ,msg.beams[3].range]).reshape(4,1)
		#new_measurement = self.outlier_rejector_dvl.compute(new_measurement)
		self.h[0] = new_measurement[0,0]
		self.h[1] = new_measurement[1,0]
		self.h[2] = new_measurement[2,0] 
		self.h[3] = new_measurement[3,0] 
	
	# Reference for speed parallel to the terrain
	def v_ref_callback(self, msg):
		self.v_ref = msg.data
		self.v_ref_t = rospy.Time.now()
			
	# Reference for distance to the terrain
	def heading_ref_callback(self, msg):
		self.heading_ref = wrap_to_pi(msg.data/180*pi)
		self.heading_ref_t = rospy.Time.now()
	
	# Reference for distance to the terrain
	def d_ref_callback(self, msg):
		self.d_ref = msg.data
		self.d_ref_t = rospy.Time.now()
			
	
	# Callback to adjust the Kp gain of the controller
	def kp_callback(self, msg):
		if self.bottom_follower is not None:
				self.bottom_follower.kp = msg.data
		
		self.kp=msg.data # for quick depth control
	
	# Callback to adjust the Ki gain of the controller NOT BEING USED
	def ki_callback(self, msg):
		self.depth_int =0
		if self.bottom_follower is not None:
				self.bottom_follower.ki = msg.data
		
		self.ki=msg.data # for quick depth control

	
	# Simple depth controller with the same inner-outer loop aproach as the bottom_following controller
	def depth_ref_callback(self, msg):
		if self.depth_time ==None:
			self.depth_time = rospy.Time.now()
			return
		# compute timings
		t = rospy.Time.now()
		Dt= (t-self.depth_time).to_sec()
		self.depth_time = t
		
		# compute error
		self.depth_ref = msg.data
		error = self.depth_ref-self.depth
		
		#compute integral
		self.depth_int = self.depth_int + Dt*error
		
		#compute controler output:
		out = -self.kp*self.depth + self.ki*self.depth_int
		self.heave_ref_pub.publish(out)
			

	"""
	Function that runs synchronously at the node frequency and where most of the node functionality is implmented
	"""
	def timerIterCallback(self, event=None):
		# Compute the time that passed since last update
		t_now = rospy.Time.now()
		Dt = (t_now - self.last_time).to_sec()
		self.last_time = t_now

		if not self.initialized:
			# if all measurements are valid
			if all(h_ is not None for h_ in self.h): 
				# initialize the estimator and also the outlier rejectors
				self.bottom_follower = BottomFollowing(self.h, self.attitude, self.alpha, self.kp, self.ki)
				self.initialized = True
				self.h = [None,None,None,None,None]
				return
				

		# if all measurements are valid    
		if all(h_ is not None for h_ in self.h): 				

			# --- Run the estimator ---
			if self.body_velocity is not None and self.attitude is not None:
				self.bottom_follower.compute(self.h, self.body_velocity, self.attitude, Dt)
				# Publish D = d * n (in inertial)
				n_hat = self.bottom_follower.kf.n
				d_hat = self.bottom_follower.kf.d
				D_I = d_hat * n_hat
				self.D_pub.publish(Vector3(D_I[0], D_I[1], D_I[2]))
			
			# --- Run the controller ---
			# check if distance reference is valid
			if self.d_ref is not None and self.d_ref_t is not  None:
				if (t_now - self.d_ref_t).to_sec() < 0.2:
					
					# check if lateral speed reference is valid
					if self.v_ref_t != None:
						if (t_now - self.v_ref_t).to_sec() > 0.2:
							self.v_ref =0
			
					# reference in inertial frame
					V_ref_I, attitude_ref = self.bottom_follower.controller(self.d_ref, self.v_ref, self.heading_ref, self.u_max) 
					
					#Rotate to the body frame 
					V_ref_B = self._rot(self.attitude[0], self.attitude[1], self.attitude[2]).T @ V_ref_I


					# publish references
					self.surge_ref_pub.publish(Float64(V_ref_B[0]))
					self.sway_ref_pub.publish(Float64(V_ref_B[1]))
					self.heave_ref_pub.publish(Float64(V_ref_B[2]))
					
					self.roll_ref_pub.publish(Float64(attitude_ref[0]*180/pi))
					self.pitch_ref_pub.publish(Float64(attitude_ref[1]*180/pi))
						
			
			# Debugging message
			self.debug_msg.altimeter = self.h[4]
			self.debug_msg.dvl_beams[0] = self.h[0]
			self.debug_msg.dvl_beams[1] = self.h[1]
			self.debug_msg.dvl_beams[2] = self.h[2]
			self.debug_msg.dvl_beams[3] = self.h[3]
			self.debug_msg.distance = float(self.bottom_follower.kf.d)
			self.debug_msg.terrain_normal_body_frame.x = float(self.bottom_follower.kf.n[0])
			self.debug_msg.terrain_normal_body_frame.y = float(self.bottom_follower.kf.n[1])
			self.debug_msg.terrain_normal_body_frame.z = float(self.bottom_follower.kf.n[2])
			self.debug_pub.publish(self.debug_msg)

					
	
	@staticmethod
	def _rot(roll, pitch, yaw): 
		cr, sr = np.cos(roll), np.sin(roll)
		cp, sp = np.cos(pitch), np.sin(pitch)
		cy, sy = np.cos(yaw), np.sin(yaw)

		R = np.array([
				[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
				[sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
				[-sp,     cp * sr,                cp * cr]
		])

		return R


def main():
    BottomFollowingNode()
    rospy.spin()

if __name__ == '__main__':
    main()
