#!/usr/bin/env python

""" 
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
"""
import rospy
from farol_bringup_algorithms.FarolSetup import FarolSetup
from std_msgs.msg import Int8, Bool

class FarolBringupVxNode():
	def __init__(self):
		"""
		Constructor for ros node
		"""

		"""
		###########################################################################################
		@.@ Init node
		###########################################################################################
		"""
		rospy.init_node('farol_bringup_vx_node')

		"""
		###########################################################################################
		@.@ Handy Variables
		###########################################################################################
		# Declare here some variables you might think usefull -> example: self.fiic = true

		"""
		self.loadParams()
		FarolSetup(self.name, self.vehicle_id, self.config_package_path, self.folder, self.namespace, self.vehicle_configuration)


	"""
	###########################################################################################
	@.@ Member Helper function to set up parameters; 
	###########################################################################################
	"""
	def loadParams(self):
		# self.node_frequency = rospy.get_param('~node_frequency', 10)
		# self.real = rospy.get_param('~real', False)
		self.name = rospy.get_param('~name', "mred")
		self.vehicle_id = str(rospy.get_param('~vehicle_id', 0))
		self.config_package_path = rospy.get_param('~config_package_path', "~/catkin_ws/farol_simulation/farol/farol_bringup")
		self.folder = rospy.get_param('~folder', "simulation")
		self.namespace = rospy.get_param('~namespace', "false")
		self.vehicle_configuration = rospy.get_param('~vehicle_configuration', None)

		#self.bags = rospy.get_param('~bags', False)
		#self.pf_controller = rospy.get_param('~path_following', "Lapierre")
		#self.waypoint = rospy.get_param('~waypoint', "standard")


def main():

	farolBringupVx = FarolBringupVxNode()
	
	# +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	rospy.spin()

if __name__ == '__main__':
	main()
