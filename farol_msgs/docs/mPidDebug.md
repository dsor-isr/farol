[farol\_msgs](index-msg.md)/mPidDebug Message
==================================================

Raw Message Definition
----------------------
```
# PID debug message  
  
# Header information  
Header header  
  
# Controller in use  
string controller  
# Reference  
float64 ref  
# Reference derivative  
float64 ref_d  
# Reference derivative filter  
float64 ref_d_filtered  
# Actual state value  
float64 state  
# Error between ref and actual state value  
float64 error  
# Error saturated  
float64 error_saturated  
  
# Feed forward term   
float64 ffTerm  
# Feed forward derivative term  
float64 ffDTerm  
# Feed forward drag term  
float64 ffDragTerm  
  
# Proportional term  
float64 pTerm  
# Integral Term  
float64 iTerm  
# Derivative Term  
float64 dTerm  
# Actual value  
float64 output  
```

Compact Message Definition
--------------------------
```
[std_msgs/Header] header  
string controller  
float64 ref  
float64 ref_d  
float64 ref_d_filtered  
float64 state  
float64 error  
float64 error_saturated  
float64 ffTerm  
float64 ffDTerm  
float64 ffDragTerm  
float64 pTerm  
float64 iTerm  
float64 dTerm  
float64 output  
```

ROS Community Messages 
--------------------------
[std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)

