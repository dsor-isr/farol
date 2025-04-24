#!/usr/bin/env python3

""" 
Developers: Ravi Regalo-> ravi.regalo@tecnico.ulisboa.pt Instituto Superior Tecnico
"""

import rospy
from farol_msgs.msg import mUSBLFix


class USBLSplitterNode():
    def __init__(self):
        rospy.init_node("usbl_splitter_node")
        self.initializePublishers()
        self.initializeSubscribers()
        self.loadParams()

    def loadParams(self):
        pass
    
    def initializeSubscribers(self):
        rospy.Subscriber(rospy.get_param('~' + "topics/subscribers/usbl_fix"), mUSBLFix, self.usbl_callback)

    def initializePublishers(self):
        self.pub_usbl_r = rospy.Publisher(rospy.get_param('~' + "topics/publishers/usbl_r"), mUSBLFix, queue_size=5)
        self.pub_usbl_be = rospy.Publisher(rospy.get_param('~' + "topics/publishers/usbl_be"), mUSBLFix, queue_size=5)

    def usbl_callback(self, msg):
        if msg.type == 0:
            self.pub_usbl_r.publish(msg)
        else:    
            self.pub_usbl_be.publish(msg)

                   
if __name__ == '__main__':
    try:
        usbl_splitter_node = USBLSplitterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
            pass