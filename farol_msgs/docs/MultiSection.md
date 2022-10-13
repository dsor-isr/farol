[farol_msgs](index-msg.md)/MultiSection Message
=====================================================

Raw Message Definition
----------------------
```
# Farol Full Path Message  
  
# Header Information  
Header header  
# Reference point  
geometry_msgs/Point RefPoint  
# Type of path  
int8  type  
# Initial Point  
geometry_msgs/Point  StartPoint  
# Center Point  
geometry_msgs/Point  CenterPoint  
# End Point  
geometry_msgs/Point  EndPoint  
# Direction of the path  
int8  adirection  
# Desired velocity  
float32 velocity  
# Gamma start  
float32 Gamma_s  
# Gamma end  
float32 Gamma_e  
```
Compact Message Definition
--------------------------
```
std_msgs/Header header  
geometry_msgs/Point RefPoint  
int8 type  
geometry_msgs/Point StartPoint  
geometry_msgs/Point CenterPoint  
geometry_msgs/Point EndPoint  
int8 adirection  
float32 velocity  
float32 Gamma_s  
float32 Gamma_e  
```

ROS Community Messages 
--------------------------
[std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)
[geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html)

