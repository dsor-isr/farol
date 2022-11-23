# static_thruster_allocation node

## In a nutshell
Transforms the forces and torques of the inner loops controllers into percentage of RPM(revolution per minute).

## Diagram
![static_thruster_allocation Diagram](img/static_thruster_allocation.png)

## Subscribers
| Subscribers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/thrust_body_request | [auv\_msgs/BodyForceRequest](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/BodyForceRequest.msg) | Thrust requested by the PID controllers |


## Publishers
| Publishers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/thrusters/rpm_command | [dsor\_msgs/Thruster](https://github.com/dsor-isr/dsor_utils/blob/d39195370a36517fc0c1a05c8e043f58720416bb/dsor_msgs/msg/Thruster.msg) | RPM command asked to the thrusters |

## Services
* None

## Parameters
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| allocation_matrix | [0.707, -0.707,  0.0, -0.306, -0.062,  0.0, <br /> 0.707,  0.707,  0.0, -0.306,  0.062,  0.0, <br /> 0.000,  0.000, -1.0, -0.087, -0.120,  0.0, <br /> 0.000,  0.000, -1.0, -0.087,  0.120,  0.0, <br /> 0.707,  0.707,  0.0,  0.300, -0.197, -0.0, <br /> 0.707, -0.707,  0.0,  0.300,  0.197, -0.0] | Forces and moments used to calculate the RPM commands |
| ctf | [0.00000177778, 0.0, 0.0] | Seabotix parameters to calculate thruster force forward |
| ctb | [-0.00000177778, 0.0, 0.0] | Seabotix parameters to calculate thruster force backward |
| max_thrust_norm | 36.0 | Maxmimum force (N) a thruster can output |
| min_thrust_norm | -36.0 | Minimum force (N) a thruster can output |
| actuators_gain | [45.0, 45.0, 45.0, 45.0, 45.0, 45.0] | Gain to scale the output thrust into the actuators |