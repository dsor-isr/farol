# Header: time & frame consistency
std_msgs/Header header     # frame_id = "map" (or your inertial frame), stamp = time this ref is valid

# Position/velocity/acceleration in WORLD frame
geometry_msgs/Point  p     # p_d
geometry_msgs/Vector3 pd   # dot{p}_d
geometry_msgs/Vector3 pdd  # ddot{p}_d

# Attitude reference (full). Use quaternion for R_d
geometry_msgs/Quaternion q # R_d

# Angular velocity/acceleration of the desired body (about its own axes):
# S(omega_d) = R_d^T * dot{R}_d, and dot{omega}_d accordingly.
geometry_msgs/Vector3 wd   # omega_d  (rad/s)
geometry_msgs/Vector3 wdd  # dot{omega}_d (rad/s^2)

bool[6] disable_axis