#!/usr/bin/env python

"""
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
"""
from __future__ import division
import rospy
import data_serializer_algorithms.DataSerializerAlgorithm as SerializerAlgorithms
import roslib.message
import std_msgs
import sys


class DataSerializerNode():
	def __init__(self):
		"""
		Constructor for ros node
		"""

		"""
		###########################################################################################
		@.@ Init node
		###########################################################################################
		"""
		rospy.init_node('data_serializer_node')

		"""
		###########################################################################################
		@.@ Dirty work of declaring subscribers, publishers and load parameters
		###########################################################################################
		"""
		self.loadParams()

		# +.+ create empty lists to accumulate serialized data and channel flag indicators
		self._serial_data = ['']*len(self._data_channels)
		self._channel_flags = [0]*len(self._data_channels)
		self._latch_count = [0]*len(self._data_channels)

		# Subscribers and Publishers use previous variables, so this is a failsafe
		self.initializeSubscribers()
		self.initializePublishers()

		"""
		###########################################################################################
		@.@ Handy Variables
		###########################################################################################
		# Declare here some variables you might think usefull -> example: self.fiic = true

		"""
        

	"""
	###########################################################################################
	@.@ Member Helper function to set up subscribers;
	###########################################################################################
	"""

	def initializeSubscribers(self):
		rospy.loginfo('Initializing Subscribers for DataSerializerNode')
		# +.+ create subscribers for all serializing/deserializing topics
		self._subs = [0]*len(self._data_channels)

		for ind, val in enumerate(self._data_channels):
			if self._vehicle_id in val['from']:
				msg_class=roslib.message.get_message_class(val['msg'])
				topic_name=val['topic_in']
				latch_qty=val['latch']
				self._subs[ind]=rospy.Subscriber(topic_name, msg_class, self.generic_callback, [ind,latch_qty])
		
		# +.+ create subscriber for from_modem topic
		self._from_modem_sub=rospy.Subscriber(self._from_modem_topic, std_msgs.msg.String, self.from_modem_callback)
		
        # +.+ create subscriber for topic that triggers output of the payload to to_modem topic
        self._trigger_sub=rospy.Subscriber(self._trigger_serialization_topic, std_msgs.msg.Empty, self.trigger_serialization_callback)
	

    """
    ###########################################################################################
    @.@ Member Helper function to set up publishers; 
    ###########################################################################################
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for DataSerializerNode')

        # +.+ create subscribers for all serializing/deserializing topics
        self._pubs=[0]*len(self._data_channels)

        for ind,val in enumerate(self._data_channels):
            if self._vehicle_id in val['to']:
                msg_class=roslib.message.get_message_class(val['msg'])
                topic_name=val['topic_out']
                self._pubs[ind]=rospy.Publisher(topic_name, msg_class, queue_size=10)
        
        # +.+ create publisher for serialized data
        self._to_modem_pub=rospy.Publisher(self._to_modem_topic, std_msgs.msg.String, queue_size=10)


    """
    ###########################################################################################
    @.@ Member Helper function to set up parameters; 
    ###########################################################################################
    """
    def loadParams(self):
        self._data_channels = rospy.get_param('~acoustic_data_channels')
        self._vehicle_id = rospy.get_param('~vehicle_id')
        # +.+ topics as parameters
        self._to_modem_topic = rospy.get_param('~topics/publishers/to_modem','to_modem')
        self._from_modem_topic = rospy.get_param('~topics/subscribers/from_modem','from_modem')
        self._trigger_serialization_topic = rospy.get_param('~topics/subscribers/trigger','trigger_serialization')

    """
    ###########################################################################################
    @.@ Callback functions/methods 
    ###########################################################################################
    """
    
    """
    ###########################################################################################
    @.@ generic_callback -> collects the info of each field in a channel
    ###########################################################################################
    """
    def generic_callback(self, msg_data,channel_info):
        channel_ind = channel_info[0]
        latch_qty = channel_info[1]
        
		# +.+ collect list of serial strings for each field in the channel
		s = []
		fields = self._data_channels[channel_ind]['fields']
		for f in fields:
			data_from_field = SerializerAlgorithms.extract_field(msg_data,f['field_name'].split('.'))
			if sys.version_info[0] == 2: 
				is_int = isinstance(data_from_field, (int, long))
			elif sys.version_info[0] == 3: 
				is_int = isinstance(data_from_field, int)
			s.append(SerializerAlgorithms.dec_to_bin(data_from_field, f['min'], f['max'], f['bits'], is_int)) 

        # +.+ set this channel as active and concatenate serial strings from all fields in the channel
        self._serial_data[channel_ind] = ''.join(s)
        self._channel_flags[channel_ind] = 1
        self._latch_count[channel_ind] = latch_qty-1
    
    """
    ###########################################################################################
    @.@ from_modem_callback -> deserializes that coming from other modems
    ###########################################################################################
    """
    def from_modem_callback(self,msg_data):
        
        tnow = rospy.Time().now()

        frame_id = msg_data.data.split(':')
        msg_data.data = frame_id[0]
        frame_id = frame_id[1]

        # +.+ decode to bit string
        msg_data.data = SerializerAlgorithms.payload_to_bits(msg_data.data)

        # +.+ separate serial payload and channel flags
        f = msg_data.data[:len(self._data_channels)]
        s = msg_data.data[len(self._data_channels):]
        c = 0

        # +.+ go through all active channels, populate messages and publish
        for ind,val in enumerate(self._data_channels):
            if f[ind]=='1' and (self._vehicle_id in val['to']): #channel is active
                # +.+ create message to populate
                msg_class = roslib.message.get_message_class(val['msg'])
                msg = msg_class()
                
                # +.+ populate header stamp if it exists
                if hasattr(msg, 'header'):
                    msg.header.stamp = tnow
                    msg.header.frame_id = frame_id

                # +.+ create dict with field values in a format accepted by fill_message_args()
				field_dict = {}
				for field in val['fields']:
					if sys.version_info[0] == 2:
						is_int = isinstance(SerializerAlgorithms.extract_field(msg,field['field_name'].split('.')), (int, long))
					elif sys.version_info[0] == 3:
						is_int = isinstance(SerializerAlgorithms.extract_field(msg,field['field_name'].split('.')), int)
					value_to_assign = SerializerAlgorithms.bin_to_dec(s[c:c+field['bits']], field['min'], field['max'], field['bits'], is_int) 
					SerializerAlgorithms.add_field_to_dict(field_dict,field['field_name'].split('.'),value_to_assign) 
					c+=field['bits'] 
                
                # populate and publish
				roslib.message.fill_message_args(msg,[field_dict])
				self._pubs[ind].publish(msg)
	
	"""
	###########################################################################################
	@.@ trigger_serialization_callback -> serializes the data/payload to transmit
	###########################################################################################
	"""
	def trigger_serialization_callback(self,msg_data):
		# +.+ put together channel flag indicators and serial data from all active channels and publish it
		f = ''.join(str(x) for x in self._channel_flags)
		s = ''.join(self._serial_data)
		self._to_modem_pub.publish(std_msgs.msg.String(SerializerAlgorithms.payload_to_bytes(f+s)))
		
		# +.+ clear serial data and channel flag indicators if latching is over
		if sys.version_info[0] == 2: 
			for i in xrange(len(self._data_channels)):
				if(self._latch_count[i]==0):
					self._serial_data[i] = ''
					self._channel_flags[i] = 0
					continue
				if(self._latch_count[i] != -1):
					self._latch_count[i] -= 1
		elif sys.version_info[0] == 3: 
			for i in range(len(self._data_channels)):
				if(self._latch_count[i]==0):
					self._serial_data[i] = ''
					self._channel_flags[i] = 0
					continue
				if(self._latch_count[i] != -1):
					self._latch_count[i] -= 1


def main():
    print ('Serializer ON')
    
    dataSerializer = DataSerializerNode()

    # +.+ going into spin; let the callbacks do all the work

    rospy.spin()

if __name__ == '__main__':
    main()
