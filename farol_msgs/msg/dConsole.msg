# Bare minimum selection of the most important data from all messages
# circulating onboard to send in a single message to the console

# nav_msgs/Odometry
float32 pose_position_x
float32 pose_position_y
float32 pose_yaw
float32 twist_linear_x
float32 twist_linear_y
float32 twist_angular_z

# delfim_msgs/Throttle
int8 throttle_port
int8 throttle_stbd

# delfim_msgs/Motor
int16 rpm_port
int16 rpm_stbd

# bits that indicate which of the above data is new
uint8 contents
uint8 THROTTLE=1
uint8 MOTOR_PORT=2
uint8 MOTOR_STBD=4
