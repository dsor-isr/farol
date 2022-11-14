# inner_loops_pid Node

## In a nutshell
This node launches the inner loop of the vehicle, where the realization is made using PID controllers. It outputs the thrust necessary for the thrusters of the vehicle.

## Diagram
![inner_loops_pid Diagram](img/inner_loops_pid.png)

## Subscribers
| Subscribers | msgs type | Purpose |
| ---         | ---       | ---     |
| /#vehicle#/nav/filter/state | [auv_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | Filtered vehicle state |
| /#vehicle#/force_bypass | [auv_msgs/BodyForceRequest](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/BodyForceRequest.msg) | Forces from external controllers to be also considered in the PID controllers |
| /#vehicle#/ref/altitude_safety | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Reference for minimum safe altitude |
| /#vehicle#/ref/depth_safety | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Reference for maximum safe depth |
| /#vehicle#/ref/\<controller> | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | References for PID controllers to follow |

where, just like in the diagram, \<controller> can be one of the following:


| Angles | Angle Rates | Speeds | Position |
| ---    | ---    | ---    | ---    |
| yaw | yaw_rate | surge | depth |
| roll | roll_rate | sway | altitude |
| pitch | - | heave | - |


## Publishers
| Publishers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/thrust_body_request | [auv_msgs/BodyForceRequest](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/BodyForceRequest.msg) | Calculated forces to be applied on the thrusters |

## Services
| Services | srv type | Purpose |
| --- | --- | --- |
| /#vehicle#/inner_forces/change_ff_gains | [inner_loops_pid/ChangeFFGains](https://github.com/dsor-isr/farol/blob/main/farol_control/inner_loops_controllers/inner_loops_pid/srv/ChangeFFGains.srv) | Change feedforward gains |
| /#vehicle#/inner_forces/change_inner_gains | [inner_loops_pid/ChangeInnerGains](https://github.com/dsor-isr/farol/blob/main/farol_control/inner_loops_controllers/inner_loops_pid/srv/ChangeInnerGains.srv) | Change the gains one of the PID controllers |
| /#vehicle#/inner_forces/change_inner_limits | [inner_loops_pid/ChangeInnerLimits](https://github.com/dsor-isr/farol/blob/main/farol_control/inner_loops_controllers/inner_loops_pid/srv/ChangeInnerLimits.srv) | Change the output limits of one of the PID controllers |

## Parameters
### Altitude Controller
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| /#vehicle#/controls/inner_loops_pid/controllers/altitude/kd | float | - | Altitude controller derivative gain |
| /#vehicle#/controls/inner_loops_pid/controllers/altitude/ki | float | - | Altitude controller integral gain |
| /#vehicle#/controls/inner_loops_pid/controllers/altitude/kp | float | - | Altitude controller proportional gain |
| /#vehicle#/controls/inner_loops_pid/controllers/altitude/max_depth | float | - | Maximum depth for altitude controller |
| /#vehicle#/controls/inner_loops_pid/controllers/altitude/max_err | float | - | Maximum error for altitude controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/altitude/max_out | float | - | Maximum output for altitude controller |
| /#vehicle#/controls/inner_loops_pid/controllers/altitude/min_err | float | - | Minimum error for altitude controller |
| /#vehicle#/controls/inner_loops_pid/controllers/altitude/min_out | float | - | Minimum output for altitude controller |

### Depth Controller
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| /#vehicle#/controls/inner_loops_pid/controllers/depth/kd | float | - | Deth controller derivative gain |
| /#vehicle#/controls/inner_loops_pid/controllers/depth/ki | float | - | Depth controller for integral gain |
| /#vehicle#/controls/inner_loops_pid/controllers/depth/kp | float | - | Depth controller for proportional gain |
| /#vehicle#/controls/inner_loops_pid/controllers/depth/max_err | float | - | Maximum error for depth controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/depth/max_out | float | - | Maximum output for depth controller |
| /#vehicle#/controls/inner_loops_pid/controllers/depth/min_alt | float | - | Maximum altitude for depth controller |
| /#vehicle#/controls/inner_loops_pid/controllers/depth/min_err | float | - | Minimum error for depth controller |
| /#vehicle#/controls/inner_loops_pid/controllers/depth/min_out | float | - | Minimum output for depth controller |

### Surge Controller
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| /#vehicle#/controls/inner_loops_pid/controllers/surge/ki | float | - | Surge controller integral gain |
| /#vehicle#/controls/inner_loops_pid/controllers/surge/kp | float | - | Surge controller proportional gain |
| /#vehicle#/controls/inner_loops_pid/controllers/surge/max_err | float | - | Maximum error for surge controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/surge/max_out | float | - | Maximum output for surge controller |
| /#vehicle#/controls/inner_loops_pid/controllers/surge/max_ref | float | - | Maximum reference for surge controller |
| /#vehicle#/controls/inner_loops_pid/controllers/surge/min_err | float | - | Minimum error for surge controller |
| /#vehicle#/controls/inner_loops_pid/controllers/surge/min_out | float | - | Minimum output for surge controller |
| /#vehicle#/controls/inner_loops_pid/controllers/surge/min_ref | float | - | Minimum reference for surge controller |

### Sway Controller
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| /#vehicle#/controls/inner_loops_pid/controllers/sway/ki | float | - | Sway controller integral gain |
| /#vehicle#/controls/inner_loops_pid/controllers/sway/kp | float | - | Sway controller proportional gain |
| /#vehicle#/controls/inner_loops_pid/controllers/sway/max_err | float | - | Maximum error for sway controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/sway/max_out | float | - | Maximum output for sway controller |
| /#vehicle#/controls/inner_loops_pid/controllers/sway/max_ref | float | - | Maximum reference for sway controller |
| /#vehicle#/controls/inner_loops_pid/controllers/sway/min_err | float | - | Minimum error for sway controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/sway/min_out | float | - | Minimum output for sway controller |
| /#vehicle#/controls/inner_loops_pid/controllers/sway/min_ref | float | - | Minimum reference for sway controller |

### Yaw Controller
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw/kd | float | - | Yaw controller derivative gain |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw/ki | float | - | Yaw controller integral gain |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw/kp | float | - | Yaw controller proportional gain |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw/max_err | float | - | Maximum error for yaw controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw/max_out | float | - | Maximum output for yaw controller |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw/min_err | float | - | Minimum error for yaw controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw/min_out | float | - | Minimum output for yaw controller |

### Yaw Rate Controller
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw_rate/ki | float | - | Yaw rate integral gain |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw_rate/kp | float | - | Yaw rate proportional gain |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw_rate/max_err | float | - | Maximum error for yaw rate controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw_rate/max_out | float | - | Maximum output for yaw rate controller |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw_rate/min_err | float | - | Minimum error for yaw rate controller input |
| /#vehicle#/controls/inner_loops_pid/controllers/yaw_rate/min_out | float | - | Minimum output for yaw rate controller |

### General Parameters
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |
| /#vehicle#/controls/inner_loops_pid/forces_hard_bypass | bool | false | If hard bypass is `true`, the controller completely ignores the yaw PID forces and reads from an external source, otherwise it sums all the force components |
| /#vehicle#/controls/inner_loops_pid/min_alt | float | - | Minimum altitude for overall controller |
| /#vehicle#/controls/inner_loops_pid/node_frequency | float | - | Working frequency of the node |
| /#vehicle#/controls/inner_loops_pid/timout_ref | float | - | Timeout for hard bypass reference use (stops using external forces after this time) |