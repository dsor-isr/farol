[farol_msgs](index-msg.md)/mPFollowingDebug Message
=========================================================


Raw Message Definition
----------------------

```yaml
# Path Following debug message  

# Header information  
Header header  

# Path Following algorithm in use  
string algorithm  
# Cross track error  
float64 cross_track_error  
# Along track error  
float64 along_track_error  
# yaw   
float64 yaw  
# psi  
float64 psi  
# Actual gamma  
float64 gamma 
```


Compact Message Definition
--------------------------

```yaml
std_msgs/Header header  
string algorithm  
float64 cross_track_error  
float64 along_track_error  
float64 yaw  
float64 psi  
float64 gamma  
```

ROS Community Messages 
--------------------------

[std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)

