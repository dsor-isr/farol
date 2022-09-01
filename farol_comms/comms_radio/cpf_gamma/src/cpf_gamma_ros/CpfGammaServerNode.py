#!/usr/bin/env python

""" 
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
"""
import rospy
from std_msgs.msg import Int8, Bool 
from farol_msgs.msg import CPFGamma
import socket
import struct
import time

class CpfGammaServerNode():
	def __init__(self):
		"""
		Constructor for ros node
		"""

		"""
		###########################################################################################
		@.@ Init node
		###########################################################################################
		"""
		rospy.init_node('cpf_gamma_server_node')

		"""
		###########################################################################################
		@.@ Handy Variables
		###########################################################################################
		"""

		# +.+ Possible messages in UDP network	
		self.messages_type = {'$CPF': 0, '$ETCPF': 1, '$ACK': 2}

		# +.+ List with methods to populate messages 
		self.data_populate = [self.cpfGammaMessage, self.etcpfGammaMessage, self.etcpfGammaMessage]
		
		self.initializePublishers()
		self.loadParams()
				
		# +.+ Socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
		self.sock.bind(("", self.port))

		# +.+ Start Server
		self.serverEnable();

	"""
	###########################################################################################
	@.@ Member Helper function to set up publishers; 
	###########################################################################################
	"""
	def initializePublishers(self):
		rospy.loginfo('Initializing Publishers for CpfGammaNode')
		self.pubs = []
		self.pubs.append(rospy.Publisher(rospy.get_param('~topics/publishers/cpf_gamma','External/Gamma'), CPFGamma, queue_size=10))
		#self.pubs.append(rospy.Publisher(rospy.get_param('~topics/publishers/etcpf_gamma','etcpf_gamma'), CPFGamma, queue_size=10))
		#self.pubs.append(rospy.Publisher(rospy.get_param('~topics/publishers/etcpf_ack','etcpf_ack'), CPFGamma, queue_size=10))
		
		

	"""
	###########################################################################################
	@.@ Member Helper function to set up parameters; 
	###########################################################################################
	"""
	def loadParams(self):
		self.node_frequency = rospy.get_param('~node_frequency', 10)
		self.id = rospy.get_param('~ID', 0)
		self.port = rospy.get_param('~broadcast_port', 2808)

	"""
	###########################################################################################
	@.@ Member Helper function to constinously run server; 
	###########################################################################################
	"""
	def serverEnable(self):
		while True:
			data, addr = self.sock.recvfrom(1024)
			self.parseData(data);

	"""
	###########################################################################################
	@.@ Parse the data received from UDP; 
	###########################################################################################
	"""
	def parseData(self, data):

		# +.+ Sends the event trigger data through UDP.
		data = data.decode()
		parsed_data = data.strip().split(',')
		if(len(parsed_data) <= 0):
			rospy.logerr("Not possible to parse data, received [{}].".format(data.strip()))
			return
		
		# This would be insane
		# self.pubs[self.messages_type[parsed_data[0]]].publish(self.data_populate[self.messages_type[parsed_data[0]]](parsed_data))
		
		# +.+ Check if the received message is part of the desired ones
		if parsed_data[0] in self.messages_type.keys():
			msg = self.data_populate[self.messages_type[parsed_data[0]]](parsed_data)

			# +.+ Check if message was populated and publish it
			if msg is not None:
				self.pubs[self.messages_type[parsed_data[0]]].publish(msg)

	"""
	###########################################################################################
	@.@ Static Helper function to populate CPFGamma messages; 
	###########################################################################################
	"""
	def cpfGammaMessage(self,parsed_data):

		# +.+ The desired length of message is 4 
		if len(parsed_data) == 4:
			# +.+ ignore our own transmissions
			if(int(parsed_data[1])==self.id):
				# rospy.logerr("Ignoring own transmissions")
				return None
	
			msg = CPFGamma()		
			msg.header.stamp = rospy.Time.now()
			msg.ID = int(parsed_data[1])
			msg.gamma = float(parsed_data[2])
			msg.vd = float(parsed_data[3])
			return msg
		else:
			return None

	"""
	###########################################################################################
	@.@ Static Helper function to populate ETCPFGamma messages; 
	###########################################################################################
	"""
	@staticmethod
	def etcpfGammaMessage(parsed_data):
		# +.+ To add in future if necessary
		return None
	"""
	###########################################################################################
	@.@ Static Helper function to populate ETCPFAck messages; 
	###########################################################################################
	"""
	@staticmethod
	def etcpfAckMessage(parsed_data):
		# +.+ To add in future if necessary
		return None

def main():

	cpfGamma = CpfGammaServerNode()

	# +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	rospy.spin()
	#while not rospy.is_shutdown():
		#continue

if __name__ == '__main__':
	main()
