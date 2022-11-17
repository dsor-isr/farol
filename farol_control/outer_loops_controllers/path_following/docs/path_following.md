# path_following Node

## In a nutshell


## Diagram
![path_following Diagram](img/path_following_diagram.png)

## Subscribers
| Subscribers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/nav/filter/state | [auv_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | Filtered state of the vehicle |
| /#vehicle#/PathData | [dsor_paths/PathData](https://dsor-isr.github.io/farol/dsor-paths/PathData/) | Path information sent from the path managing node |
| /#vehicle#/PF/vc | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Velocity correction sent from the CPF node (if deployed) |
| /#vehicle#/Flag | [std_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html) | Flag that determines teh state of the vehicle (idle, following waypoint, path, etc.) |


## Publishers
| Publishers | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/Gamma | | |
| /#vehicle#/pfollowing/debug | | |
| /#vehicle#/current/x | | |
| /#vehicle#/current/y | | |
| /#vehicle#/ref/`<controller>` | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | |

## Services
| Services | msg type | Purpose |
| --- | --- | --- |
| | | |

## Parameters
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |