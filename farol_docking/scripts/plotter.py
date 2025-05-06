#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np


class KalmanPlotter:
    def __init__(self):
        rospy.init_node('kalman_plotter', anonymous=True)

        # ROS Subscriptions
        rospy.Subscriber('/kalman_filter/pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/kalman_filter/depth', Float64, self.depth_callback)
        rospy.Subscriber('/kalman_filter/heading', Float64, self.heading_callback)

        # Data storage
        self.x, self.y, self.depth, self.heading = [], [], [], []
        self.x_pred, self.y_pred, self.depth_pred, self.heading_pred = [], [], [], []
        self.cov = None

        # Initialize plot
        self.fig, (self.ax2d, self.ax_depth, self.ax_heading) = plt.subplots(3, 1, figsize=(10, 15))
        plt.ion()
        self.fig.show()
        self.fig.canvas.draw()

    def pose_callback(self, msg):
        # Extract data
        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)
        self.x_pred.append(msg.pose.pose.position.x)  # Replace with actual predicted data
        self.y_pred.append(msg.pose.pose.position.y)  # Replace with actual predicted data
        self.cov = np.array(msg.pose.covariance).reshape((6, 6))[:2, :2]  # 2D covariance matrix

        self.update_2d_plot()

    def depth_callback(self, msg):
        self.depth.append(msg.data)
        self.depth_pred.append(msg.data)  # Replace with actual predicted data
        self.update_depth_plot()

    def heading_callback(self, msg):
        self.heading.append(msg.data)
        self.heading_pred.append(msg.data)  # Replace with actual predicted data
        self.update_heading_plot()

    def update_2d_plot(self):
        self.ax2d.clear()
        self.ax2d.plot(self.x, self.y, 'b-', label="Actual")
        self.ax2d.plot(self.x_pred, self.y_pred, 'r--', label="Predicted")

        if self.cov is not None:
            # Draw covariance ellipse
            eigvals, eigvecs = np.linalg.eig(self.cov)
            angle = np.degrees(np.arctan2(eigvecs[0, 1], eigvecs[0, 0]))
            width, height = 2 * np.sqrt(eigvals)  # Scale factor for covariance
            ellipse = Ellipse((self.x[-1], self.y[-1]), width, height, angle, alpha=0.3, color='gray')
            self.ax2d.add_patch(ellipse)

        self.ax2d.set_title("2D Position")
        self.ax2d.set_xlabel("X Position")
        self.ax2d.set_ylabel("Y Position")
        self.ax2d.legend()
        self.fig.canvas.draw()

    def update_depth_plot(self):
        self.ax_depth.clear()
        self.ax_depth.plot(self.depth, 'b-', label="Actual")
        self.ax_depth.plot(self.depth_pred, 'r--', label="Predicted")
        self.ax_depth.fill_between(range(len(self.depth)), 
                                   np.array(self.depth) - 0.1, 
                                   np.array(self.depth) + 0.1, 
                                   color='gray', alpha=0.3)
        self.ax_depth.set_title("Depth")
        self.ax_depth.set_xlabel("Time")
        self.ax_depth.set_ylabel("Depth")
        self.ax_depth.legend()
        self.fig.canvas.draw()

    def update_heading_plot(self):
        self.ax_heading.clear()
        self.ax_heading.plot(self.heading, 'b-', label="Actual")
        self.ax_heading.plot(self.heading_pred, 'r--', label="Predicted")
        self.ax_heading.fill_between(range(len(self.heading)), 
                                     np.array(self.heading) - 0.1, 
                                     np.array(self.heading) + 0.1, 
                                     color='gray', alpha=0.3)
        self.ax_heading.set_title("Heading")
        self.ax_heading.set_xlabel("Time")
        self.ax_heading.set_ylabel("Heading")
        self.ax_heading.legend()
        self.fig.canvas.draw()

    def run(self):
        while not rospy.is_shutdown():
            plt.pause(0.01)


if __name__ == '__main__':
    try:
        plotter = KalmanPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
