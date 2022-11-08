# filter Node

## In a nutshell
A general-purpose kalman filter for vehicle state estimation. For indepth documentation, refer the links below:
* [Theory](./theory.md) 
* [User Guide](./user_guide.md)
* [Developer Notes](./developer_notes.md) 

## Diagram
![Sensor Fusion Diagram](img/filter.png)

## Subscribers
| Subscribers                       | msgs type                                                                 | Purpose                                             |
| -----------                       | --------------                                                             | ---------                                           |
| /#vehicle#/Flag                    | [std\_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html) | |
| /#vehicle#/measurement/orientation | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | Sensor measurement of vehicle orientation |
| /#vehicle#/measurement/position    | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | Sensor measurement of vehicle position |
| /#vehicle#/measurement/velocity    | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | Sensor measurement of vehicle velocity along body-frame |
| /#vehicle#/nav/filter/reset        | [std\_msgs/Empty](http://docs.ros.org/en/api/std_msgs/html/msg/Empty.html) | Empty messages sent to reset the navigation filter |
| /#vehicle#/nav/filter/state        | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | |
| /tf                               | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | |
| /tf\_static                       | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | |

## Publishers
| Publishers                     | msgs type                            | Purpose                                                      |
| -----------                    | --------------                       | ---------                                                    |
| /#vehicle#/State\_dr            | [farol\_msgs/mState](https://github.com/dsor-isr/farol/blob/main/farol_msgs/msg/mState.msg) | |
| /#vehicle#/nav/filter/currents  | [farol\_msgs/Currents](https://github.com/dsor-isr/farol/blob/main/farol_msgs/msg/Currents.msg) | |
| /#vehicle#/nav/filter/state     | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | |
| /#vehicle#/nav/filter/state\_dr | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | |
| /tf                            | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | |

## Services
| Services                        | msgs type                           | Purpose                                     |
| -----------                     | --------------                      | ---------                                   |
| /#vehicle#/nav/reset\_filter\_dr | [std\_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html) |                                             |

## Parameters
| Parameters                                                              | type   | Default     | Purpose                       |
| -----------                                                             | ----   | ----------  | ---------                     |
| /#vehicle#/nav/filter/dvl/body\_frame                                   | bool   | true        | |
| /#vehicle#/nav/filter/kalman\_filter/bypass\_ahrs                       | bool   | false       | |
| /#vehicle#/nav/filter/kalman\_filter/manually\_initialization/frame\_id | string | gnss        | |
| /#vehicle#/nav/filter/kalman\_filter/manually\_initialization/noise     | array  | [1.0, 1.0, 0.01, 0.001, 0.001, 0.01, 0.02, 0.03, 0.04, 0.01, 0.01, 0.01, 0.0, 0.0, 0.01] | |
| /#vehicle#/nav/filter/kalman\_filter/manually\_initialization/value     | array | [4290794.432828665, 491936.5610790758, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0.0, 0.0, 0.0, 0.0, 5.0] | |
| /#vehicle#/nav/filter/kalman\_filter/outlier\_rejection\_threshold      | int | 15 | |
| /#vehicle#/nav/filter/kalman\_filter/predict\_period                    | float | 0.1 | |
| /#vehicle#/nav/filter/kalman\_filter/process\_covariance                | array | [1.0, 0.1, 0.1, 0.1, 0.1, 0.1] | |
| /#vehicle#/nav/filter/kalman\_filter/reject\_counter                    | array | [8, 200, 12, 4, 7, 6] | |
| /#vehicle#/nav/filter/kalman\_filter/reset\_period                      | int | 0 | |
| /#vehicle#/nav/filter/kalman\_filter/save\_measurement\_interval        | int | 5 | |
| /#vehicle#/nav/filter/kalman\_filter/sensors                            |
| /#vehicle#/nav/filter/name\_vehicle\_id                                 | string | /#vehicle#/ | |
| /#vehicle#/nav/filter/node\_frequency                                   | float | 10 | |
| /#vehicle#/nav/filter/originLat                                         |
| /#vehicle#/nav/filter/originLon                                         |
| /#vehicle#/nav/filter/services\_dr/reset\_filter\_dr                    | string | /#vehicle#/nav/reset\_filter\_dr | |
| /#vehicle#/nav/filter/tf/broadcast                                      | bool | true | |
| /#vehicle#/nav/filter/tf/frames/base\_link                              | string | base\_link | |
| /#vehicle#/nav/filter/tf/frames/map                                     | string | map | |
| /#vehicle#/nav/filter/tf/frames/odom                                    | string | null | |
| /#vehicle#/nav/filter/tf/frames/world                                   | string | map | |