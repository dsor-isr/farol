[farol\_msgs](index-msg.md)/Formation Message
=================================================

Raw Message Definition
----------------------

```yaml
# Formation vehicle message  

# Header information  
Header header  

# vehicle id  
int8 ID  
# Towards the path  
float64 x    
# Perpendicular left from x  
float64y    
# Gamma end for all sections  
float64 Gamma_e   
# Length for all sections  
float64 Length 
```

Compact Message Definition
--------------------------

```yaml
std_msgs/Header header  
int8 ID  
float64 x  
float64 y  
float64 Gamma_e  
float64 Length  
```

ROS Community Messages 
--------------------------

[std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)
