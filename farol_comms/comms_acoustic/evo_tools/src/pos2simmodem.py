#!/usr/bin/env python
#
# Creation:
#   February 2014
#
#
# Description:
#   This code is intended for reading vehicle position from odometry and send it
#   to the simulated modems provided by CMRE over a VPN.

# ROS basics
import roslib
import rospy

# ROS messsages
from nav_msgs.msg import Odometry
from auv_msgs.msg import NavigationStatus
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion 
import time

# Internet communications
import socket

#===============================================================================
class Pos2SimModem(object):
    '''
    Class to hold the read of Odometry messages and send position to the
    simulated modem.
    '''

    #===========================================================================
    def __init__(self):
        '''
        Initializing the necessary variables.
        '''
        # Parameters
        ip = rospy.get_param('~ip', '10.42.10.9')
        rospy.loginfo(ip)
        port = rospy.get_param('~port', 11000)
        rospy.loginfo(port)
        self.vehicle_name = rospy.get_param('~vehicle_name')
        rospy.loginfo(self.vehicle_name)
        self.address = (ip, port)


        # Socket connection
        #self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
       # self.sock.connect(self.address)
        rospy.loginfo("\nPos2SimModem:\n\tConnected to %s:%d", ip, port)

        # Time
        self.time = rospy.Time.now()

        # Subscriber
        # For farol simulation
        self.sub_sim = rospy.Subscriber(rospy.get_param('~' + "topics/subscribers/position_sim"),
                                         Odometry, self.position_callback)
        # For gazebo
        self.sub_gazebo = rospy.Subscriber(rospy.get_param('~' + "topics/subscribers/position_gazebo"),
                                         ModelStates, self.position_callback)

    #===========================================================================
    def position_callback(self, msg):

        input_message_type = str(msg._type)

        '''
        Reads Odometry and sends position.
        '''

        if (rospy.Time.now() - self.time).to_sec() < 0.5:
            return

        # Update time
        self.time = rospy.Time.now()
        
        # Fetch postions depending on the type of message used
        if(input_message_type == 'nav_msgs/Odometry'):
            data = "%.2f %.2f %.2f\n" % (msg.pose.pose.position.x, msg.pose.pose.position.y, -msg.pose.pose.position.z); 
        elif(input_message_type == 'gazebo_msgs/ModelStates'):
            if self.vehicle_name in msg.name:
                index = msg.name.index(self.vehicle_name)
                # data = "%.2f %.2f %.2f\n" % (msg.pose[index].position.y, msg.pose[index].position.x, msg.pose[index].position.z)
                (roll, pitch, yaw) = euler_from_quaternion([msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z, msg.pose[index].orientation.w])
                print(roll, -pitch, -yaw+1.570796327)
                data = "%.2f %.2f %.2f %.2f %.2f %.2f \n" % (msg.pose[index].position.y, msg.pose[index].position.x, msg.pose[index].position.z, roll, -pitch, -yaw+1.570796327)
                #print(self.vehicle_name,":",position)
        

        # Send
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.address)
            self.sock.sendall(data.encode('utf-8'))
            self.sock.close()
        except Exception as e:
            rospy.logwarn('%s : %s : error sending position to modem', rospy.get_name(), e)
            self.sock.close()

# THERE IS AN ERROR WITH THE MESSAGE FORMAT !!!

#===============================================================================
if __name__ == '__main__':

    # Start sending
    rospy.init_node('Pos2SimModem')
    rospy.loginfo('Start Pos2simmodem')
    node = Pos2SimModem()
    rospy.spin()