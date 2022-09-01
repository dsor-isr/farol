#!/usr/bin/env python

""" 
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
"""
import rospy
from std_msgs.msg import Int8, Bool
from farol_msgs.msg import CPFGamma
import socket

class CpfGammaClientNode():
	def __init__(self):
		"""
		Constructor for ros node
		"""

		"""
		###########################################################################################
		@.@ Init node
		###########################################################################################
		"""
		rospy.init_node('cpf_gamma_client_node')


		"""
		###########################################################################################
		@.@ Dirty work of declaring subscribers, publishers and load parameters 
		###########################################################################################
		"""
		self.loadParams()

		# Tuple with Broadcast address and port
		self.address = (self.addr, self.port)

		# Socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.bind(("",0))
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

		self.initializeSubscribers()

		"""
	###########################################################################################
	@.@ Member Helper function to set up subscribers; 
	###########################################################################################
	"""
	def initializeSubscribers(self):
		rospy.loginfo('Initializing Subscribers for CpfGammaNode')
		rospy.Subscriber(rospy.get_param('~topics/subscribers/internal_gamma', 'Internal/Gamma'), CPFGamma, self.broadcastGammaCallback)	

	
	"""
	###########################################################################################
	@.@ Member Helper function to set up parameters; 
	###########################################################################################
	"""
	def loadParams(self):
		self.node_frequency = rospy.get_param('~node_frequency', 10)
		self.port = rospy.get_param('~broadcast_port', 2808)
		self.addr = rospy.get_param('~broadcast_address', '127.255.255.255')


	"""
	###########################################################################################
	@.@ Callback functions/methods 
	###########################################################################################
	"""

	# Broadcasts the value of gamma from path following
	def broadcastGammaCallback(self, msg):

		# Build the message
		data = "$CPF,%02i,%06.4f,%06.4f\n" % (msg.ID, msg.gamma,msg.vd)

		# print(data)
		# Send message
		count = 0
		while count < len(data):
			count += self.sock.sendto(data.encode(), self.address)

def main():
	
	cpfGamma = CpfGammaClientNode()

	# Added to work with timer -> going into spin; let the callbacks do all the work
	rospy.spin()

if __name__ == '__main__':
	main()
