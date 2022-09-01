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
        ip = rospy.get_param('~ip', '10.42.10.1')
        port = rospy.get_param('~port', 11000)
        self.address = (ip, port)

        # Socket connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.sock.connect(self.address)
        rospy.loginfo("\nPos2SimModem:\n\tConnected to %s:%d", ip, port)

        # Time
        self.time = rospy.Time.now()

        # Subscriber
        self.sub_odom = rospy.Subscriber(rospy.get_param('~' + "topics/subscribers/position"),
                                         Odometry, self.position_callback)

    #===========================================================================
    def position_callback(self, msg):
        '''
        Reads Odometry and sends position.
        '''
        # Check time
        #rospy.loginfo("stamp %.2f time %.2f diff %.2f" % (msg.header.stamp.to_sec(),
        #                                        self.time.to_sec(),
        #                                        (msg.header.stamp -
        #                                        self.time).to_sec()))
        #rospy.loginfo('lalalalalal')
        if (msg.header.stamp - self.time).to_sec() < 0.5:
            return

        # Update time
        self.time = msg.header.stamp

        # Prepare message
        # data = "%.2f %.2f %.2f\n" % (msg.pose.pose.position.x - 4288741,
        #                              msg.pose.pose.position.y - 492665,
        #                             -msg.pose.pose.position.z)
        data = "%.2f %.2f %.2f\n" % (msg.pose.pose.position.x, msg.pose.pose.position.y, -msg.pose.pose.position.z); 
        # rospy.loginfo("Pos to modem ["+data+"]")
        # Send
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.address)
            self.sock.sendall(data.encode('utf-8'))
            self.sock.close()
        except Exception as e:
            rospy.logwarn('%s : %s : error sending position to modem', rospy.get_name(), e)
            self.sock.close()

#===============================================================================
if __name__ == '__main__':

    # Start sending
    rospy.init_node('Pos2SimModem')
    node = Pos2SimModem()
    rospy.spin()
