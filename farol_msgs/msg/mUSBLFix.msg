# The USBL fix message
# Valid range only
int32 RANGE_ONLY = 0
# Valid azimuth
int32 AZIMUTH_ONLY = 1
# Valid position (usually assumes valid range and azimuth)
int32 FULL_FIX = 2
# Relative Position (usually assumes valid range and azimuth)
int32 CARTESIAN = 3

# Header information
Header header

# Beacon ID and Name - remove source_name if possible
int32 source_id
string source_name
string source_frame_id

# Fix type
int32 type

# Relative position in inertial
geometry_msgs/Point relative_position

# Range to target
float32 range
# Bearing to target
float32 bearing
# Elevation to target
float32 elevation
# Sound speed use for calculation
float32 sound_speed

# Raw angles
# Bearing to target
float32 bearing_local
# Elevation to target
float32 elevation_local 

# Body angles
# Bearing to target
float32 bearing_body
# Elevation to target
float32 elevation_body 

# Rotation angles of the local device, in radians,floating point.
float32 ahrs_roll
float32 ahrs_pitch
float32 ahrs_yaw


# Measurement Noise in relative_position
float64[9] position_covariance