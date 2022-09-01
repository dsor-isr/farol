Header header
uint8 GPS_Good # 0 = Auto; 1 = DGPS; 2 = RTK Float; 3 = RTK Fixed; 255 = no GPS
uint8 IMU_Good # 0 = Bad; 1 = Good/Not Filtered; 2= ; 3 = Very good
float64 Depth
float64 X
float64 Y
float64 Z
float64 Vx
float64 Vy
float64 Vz   # Not using
float64 u    # Surge
float64 Yaw
float64 Pitch
float64 Roll
float64 Yaw_rate
float64 Pitch_rate
float64 Roll_rate
float64 In_Press
float64 In_Press_dot
uint8 battery_level
float64 altitude
uint8 status
uint8 STATUS_FAULT = 0
uint8 STATUS_HPOSITION_OK = 1
uint8 STATUS_VPOSITION_OK = 2
uint8 STATUS_ORIENTATION_OK = 4
uint8 STATUS_VELOCITY_OK = 8
uint8 STATUS_ALL_OK = 15
