# auvstate2mstate Node

## In a nutshell
This node converts state in *auv_msgs::NavigationStatus* to state in *farol_msgs::mState*. Also need *inside_pressure* data for mState.

## Diagram
![AuvState2mState Diagram](img/auvstate2mstate.png)

## Subscribers
| Subscribers                          | msgs type                                                                                                                                        | Purpose                                             |
| -----------                          | --------------                                                                       | ---------                                                                         |
| /#vehicle#/nav/filter/state          | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | The navigation state of the vehicle after filtering |
| /#vehicle#/nav/inside\_pressure/data | [std\_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html)                                                                   | Information about pressure over the vehicle         |
| /#vehicle#/sensors/gnss              | [sensor\_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)                                                         | Sensor gnss reading                                 |

## Publishers
| Publishers       | msgs type                                                                         | Purpose                                                      |
| -----------      | --------------                                                                    | ---------                                                    |
| /#vehicle#/State | [farol\_msgs/mState](https://dsor-isr.github.io/farol/farol-ros-messages/mState/) | The filtered state of the vehicle turned into mState message |

## Services
* None

## Parameters
| Parameters                 | type   | Default     | Purpose                       |
| -----------                | ----   | ----------  | ---------                     |
| /#vehicle#/node\_frequency | float  | 10.0        | Working frequency of the node |
