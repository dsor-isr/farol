[farol\_msgs](index-msg.md)/Section Message
================================================

Raw Message Definition
----------------------
```yaml
# Farol Path Section Type Message   
  
# Hader information  
Header Header  
# All other points are relative to this  
float64 xrefpoint  
float64 yrefpoint  
  
# Starting point  
float64 xs            # start  
float64 ys  
  
# Center point  
float64 xc            # center  
float64 yc  
  
# End point  
float64 xe            # end  
float64 ye  
  
# Desired velocity  
float64 Vl              
  
# Direction of the arc  
float64 direction       
# Radius of the arc  
float64 R0  
  
# Gamma start  
float64 Gamma_s  
# Gamma end  
float64 Gamma_e  
```

Compact Message Definition
--------------------------
```
std_msgs/Header Header  
float64 xrefpoint  
float64 yrefpoint  
float64 xs  
float64 ys  
float64 xc  
float64 yc  
float64 xe  
float64 ye  
float64 Vl  
float64 direction  
float64 R0  
float64 Gamma_s  
float64 Gamma_e  
```

ROS Community Messages 
--------------------------
[std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)
