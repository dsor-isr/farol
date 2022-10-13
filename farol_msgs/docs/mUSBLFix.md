[farol_msgs](index-msg.md)/mUSBLFix Message
=================================================

Raw Message Definition
----------------------
```yaml
# The USBL fix message  
# Valid range only  
int32 RANGE_ONLY = 0  
# Valid azimuth  
int32 AZIMUTH_ONLY = 1  
# Valid position (usually assumes valid range and azimuth)  
int32 FULL_FIX = 2  
# Relative Position (usually assumes valid range and azimuth)  
int32 CARTESIAN = 3  
  
# Header information  
Header header  
  
# Beacon ID and Name - remove source_name if possible  
int32 source_id  
string source_name  
string source_frame_id  
  
# Fix type  
int32 type  
  
# Relative position in inertial  
geometry_msgs/Point relative_position  
  
# Range to target  
float32 range  
# Bearing to target  
float32 bearing  
# Elevation to target  
float32 elevation  
# Sound speed use for calculation  
float32 sound_speed  
  
# Raw angles  
# Bearing to target  
float32 bearing_raw  
# Elevation to target  
float32 elevation_raw   
  
# Measurement Noise in relative_position  
float64 position_covariance  
```
Compact Message Definition
--------------------------
```
int32 RANGE_ONLY=0  
int32 AZIMUTH_ONLY=1  
int32 FULL_FIX=2  
int32 CARTESIAN=3  
std_msgs/Header header  
int32 source_id  
string source_name  
string source_frame_id  
int32 type  
geometry_msgs/Point relative_position  
float32 range  
float32 bearing  
float32 elevation  
float32 sound_speed  
float32 bearing_raw  
float32 elevation_raw  
float64 position_covariance  
```
ROS Community Messages 
--------------------------
[std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)
[geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html)
