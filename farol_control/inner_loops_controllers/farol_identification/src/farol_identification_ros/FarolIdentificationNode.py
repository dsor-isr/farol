#!/usr/bin/env python

""" 
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
"""
import rospy
import numpy as np
from auv_msgs.msg import BodyForceRequest

class FarolIdentificationNode():
    def __init__(self):
        rospy.init_node('logitech_extreme_node')
        self.initializePublishers()

    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for FarolIdentificationNode')
        self.all_pub = rospy.Publisher(rospy.get_param('~topics/publishers/allocation'), BodyForceRequest, queue_size=10)

    def headingIdent(self):
        aux = BodyForceRequest()
        aux.wrench.force.x=0.0
        aux.wrench.force.y=0.0
        aux.wrench.force.z=0.0
        aux.wrench.torque.x=0.0
        aux.wrench.torque.y=0.0
        aux.wrench.torque.z=0.0

        for i in range(5):
            rospy.loginfo('1 N.m')
            aux.wrench.torque.z=1.0
            self.all_pub.publish(aux) 
            rospy.sleep(5.)
            rospy.loginfo('Stop')
            aux.wrench.torque.z=0.0
            self.all_pub.publish(aux) 
            rospy.sleep(5.)

        for i in range(10):
            rospy.loginfo(str(0.2*i)+"N.m")
            aux.wrench.torque.z+=0.2
            self.all_pub.publish(aux) 
            rospy.sleep(5.)
    
        for i in range(5):
            rospy.loginfo('-1 N.m')
            aux.wrench.torque.z=1.0
            self.all_pub.publish(aux) 
            rospy.sleep(5.)
            rospy.loginfo('Stop')
            aux.wrench.torque.z=0.0
            self.all_pub.publish(aux) 
            rospy.sleep(5.)

        for i in range(10):
            rospy.loginfo("-"+str(0.2*i + 0.2)+"N.m")
            aux.wrench.torque.z+=0.2
            self.all_pub.publish(aux) 
            rospy.sleep(5.)

        aux.wrench.torque.z=0.0
        self.all_pub.publish(aux) 


def main():
    print 'success'
    farol_ident = FarolIdentificationNode()
    farol_ident.headingIdent()
    rospy.spin()

if __name__ == '__main__':
    main()
