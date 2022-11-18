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
| /#vehicle#/Gamma | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Vehicle progression relative to the path parameter |
| /#vehicle#/pfollowing/debug | [farol_msgs](https://dsor-isr.github.io/farol/farol-ros-messages/mPFollowingDebug/) | Topic used for debugging purposes only |
| /#vehicle#/current/x | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Observe x current component |
| /#vehicle#/current/y | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Observe y current component |
| /#vehicle#/ref/`<controller>` | [std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | References to be given to the inner loops |

## Services
| Services | msg type | Purpose |
| --- | --- | --- |
| /#vehicle#/PFStart | [path_following/StartPF](StartPF.md) | Run the previously established path following |
| /#vehicle#/PFStop | [path_following/StopPF](StopPF.md) | |
| /#vehicle#/PFUpdateGains | [path_following/UpdateGainsPF](UpdateGainsPF.md) | |
| /#vehicle#/ResetVT | [path_following/ResetVT](ResetVT.md) | |
| /#vehicle#/PFSetRelativeHeading | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/PFSetMarcelo | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/PFSetAguiar | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/PFSetBrevik | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/PFSetFossen | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/PFSetRomulo | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/PFSetLapierre | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/PFSetPramod | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/PFSetSamson | [path_following/SetPF](SetPF.md) | |
| /#vehicle#/ResetPath | [path_following/StartPF](StartPF.md) | |
| /#vehicle#/SetMode | [path_following/StartPF](StartPF.md) | |
| /#vehicle#/controls/send_wp_standard | [path_following/StartPF](StartPF.md) | |

## Parameters
| Parameters | type | Default | Purpose |
| --- | --- | --- | --- |