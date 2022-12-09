# open_loop_controller

## In a nutshell
The open\_loop\_controller node is, like it suggests, can be used as an alternative to the typical PID controllers implemented on the vehicles, depending only on the current input reference values.

## Diagram
![open\_loop](img/open_loop.png)

## Subscribers
| Subscribers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/ref/surge | [std\_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Surge reference for vehicle to follow |
| /#vehicle#/ref/sway | [std\_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Sway reference for vehicle to follow |
| /#vehicle#/ref/heave | [std\_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Heave reference for vehicle to follow |
| /#vehicle#/ref/yaw\_rate | [std\_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Yaw rate for vehicle to follow |

## Publishers
| Publishers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/thrust\_body\_request | [auv\_msgs/BodyForceRequest](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/BodyForceRequest.msg) | Forces requested by the controller to the thrusters |

## Services
* None

## Parameters
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| /#vehicle#/controls/ope\_loop\_controller/gain\_Fx | float | - | Gain for the force component in x |
| /#vehicle#/controls/open\_loop\_controller/gain\_Fy | float | - | Gain for the force component in y |
| /#vehicle#/controls/open\_loop\_controller/gain\_Fz | float | - | Gain for the force component in z |
| /#vehicle#/controls/open\_loop\_controller/gain\_Tz | float | - | Gain for the torque component in z |
| /#vehicle#/controls/open\_loop\_controller/node\_frequency | float | 10 | Working frequency of the node |