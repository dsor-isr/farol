[farol_msgs](index-msg.md)/CPFGamma Message
=================================================

Raw Message Definition
----------------------

```yaml
# CPF gamma message
# Header information  
Header header      

# The ID corresponding to the vehicle  
uint8 ID           

# The value of the virtual target of that vehicle  
float64 gamma      

# The desired speed for that virtual target  
float64 vd        
```



Compact Message Definition
--------------------------

```yaml
std_msgs/Header header  
uint8 ID  
float64 gamma  
float64 vd
```

 

ROS Community Messages 
--------------------------

[std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)
