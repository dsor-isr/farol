# Medusa State message

# Header Information
Header header

# GPS Quality
uint8 GPS_Good # 0 = Auto; 1 = DGPS; 2 = RTK Float; 3 = RTK Fixed; 255 = no GPS
# IMU Quality
uint8 IMU_Good # 0 = Bad; 1 = Good/Not Filtered; 2= ; 3 = Very good

# Depth Measurmente
float64 Depth

# East 
float64 X
#North
float64 Y
# Down
float64 Z
# Velocity in X
float64 Vx
# Velocity in Y
float64 Vy
# Velocity in Z
float64 Vz   # Not using
# Surge
float64 u    # Surge
# Yaw
float64 Yaw
# Pitch
float64 Pitch
# Roll
float64 Roll
# Yaw Rate
float64 Yaw_rate
# Pitch Rate
float64 Pitch_rate
# Roll Rate
float64 Roll_rate
# Inside Pressure
float64 In_Press
# Inside Pressure rate
float64 In_Press_dot
# Battery level
uint8 battery_level
# Altitude
float64 altitude

# Status of the vehicle
uint8 status
uint8 STATUS_FAULT = 0
uint8 STATUS_HPOSITION_OK = 1
uint8 STATUS_VPOSITION_OK = 2
uint8 STATUS_ORIENTATION_OK = 4
uint8 STATUS_VELOCITY_OK = 8
uint8 STATUS_ALL_OK = 15
