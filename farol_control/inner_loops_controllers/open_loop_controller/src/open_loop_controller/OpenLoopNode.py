#!/usr/bin/env python3

"""
@author: Marcelo Fialho Jacinto
@email: marcelo.jacinto@tecnico.ulisboa.pt
@date: 30/03/2022
@licence: MIT
"""
import rospy
import numpy as np
from std_msgs.msg import Float64
from auv_msgs.msg import BodyForceRequest

class OpenLoopNode:
    """
    Remote controller ROS node class. Receives from a joystick (using pygame) the desired controls for
    the inner-loops and publishes these periodically (at a pre-defined frequency) to the inner-loops of the
    vehicle
    """

    def __init__(self):
        """
        Class constructor. Initializes the ros node, loads the parameters from the ros parameter server, creates the inner-loops
        publishers and initializes the timer that publishes the desired inputs to the inner-loops
        """

        # ---Initialize the ROS NODE---
        rospy.init_node('open_loop_controller')

        # --- Load parameters from the ROS parameter server
        self.load_params()

        # --- Initialize subscribers and publishers
        self.initializeSubscribers()
        self.initializePublishers()

        self.h_timerActivate = False

        # --- Desired variables to apply to the vehicle (NOTE: this is not speeds but rather forces and torques to applyt to the vehicle)
        self.surge_desired = 0.0       # Fx (force on x-axis)
        self.sway_desired = 0.0        # Fy (force on y-axis)
        self.heave_desired = 0.0       # Fz (force on z-axis)
        self.yaw_rate_desired = 0.0    # Tz (torque about the z-axis)

        # Last time instant where a message with desired speeds was received
        self.last_update = rospy.Time.now()
        
        # ---Start the ROS NODE callback---
        self.initializeTimer()


    def load_params(self):

        # Read the node frequency
        self.node_frequency = rospy.get_param('~node_frequency', 10)

        # Read the configurations from the ros parameter server
        self.gain_Fx = rospy.get_param('~gain_Fx')
        self.gain_Fy = rospy.get_param('~gain_Fy')
        self.gain_Fz = rospy.get_param('~gain_Fz')
        self.gain_Tz = rospy.get_param('~gain_Tz')

    def initializeSubscribers(self):
        self.surge_sub = rospy.Subscriber(rospy.get_param('~topics/subscribers/surge_ref'), Float64, self.surgeCallback)
        self.sway_sub = rospy.Subscriber(rospy.get_param('~topics/subscribers/sway_ref'), Float64, self.swayCallback)
        self.heave_sub = rospy.Subscriber(rospy.get_param('~topics/subscribers/heave_ref'), Float64, self.heaveCallback)
        self.yaw_rate_sub = rospy.Subscriber(rospy.get_param('~topics/subscribers/yaw_rate_ref'), Float64, self.yawRateCallback)
    
    def initializePublishers(self):
        self.force_pub = rospy.Publisher(rospy.get_param('~topics/publishers/body_force_request'), BodyForceRequest, queue_size=1)

    def timerCallback(self, event):
        """
        Callback used to publish to the inner-loops the desired control inputs
        :param event: A timer event - unused but required by the ROS API
        """

        # Scale the desired input (surge, sway, heave and yaw-rate to some force and torque)
        output_msg = BodyForceRequest()
        output_msg.header.stamp = rospy.Time.now()

        output_msg.wrench.force.x = self.gain_Fx * self.surge_desired
        output_msg.wrench.force.y = self.gain_Fy * self.sway_desired
        output_msg.wrench.force.z = self.gain_Fz * self.heave_desired

        output_msg.wrench.torque.x = 0.0
        output_msg.wrench.torque.y = 0.0
        output_msg.wrench.torque.z = self.gain_Tz * self.yaw_rate_desired
        
        self.force_pub.publish(output_msg)

        
    # ---- Callbacks section ----

    def surgeCallback(self, msg: Float64):
        self.surge_desired = float(msg.data)
        self.last_update = rospy.Time.now()

    def swayCallback(self, msg: Float64):
        self.sway_desired = float(msg.data)
        self.last_update = rospy.Time.now()

    def heaveCallback(self, msg: Float64):
        self.heave_desired = float(msg.data)
        self.last_update = rospy.Time.now()

    def yawRateCallback(self, msg: Float64):
        self.yaw_rate_desired = float(msg.data)
        self.last_update = rospy.Time.now()
       
    def initializeTimer(self):
        """
        Method that starts the system timer that periodically calls a callback
        """
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timerCallback)

def main():
    """
    Initialize the RemoteControllerNode and let the timer callback do all the work
    """
    open_loop_controller = OpenLoopNode()
    rospy.spin()

if __name__ == '__main__':
    main()



