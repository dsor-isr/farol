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
| /#vehicle#/Flag                    | [std\_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html) | Farol stack state machine (waypoint, idle, path following, etc.) |
| /#vehicle#/measurement/orientation | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | Sensor measurement of vehicle orientation |
| /#vehicle#/measurement/position    | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | Sensor measurement of vehicle position |
| /#vehicle#/measurement/velocity    | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | Sensor measurement of vehicle velocity along body-frame |
| /#vehicle#/nav/filter/reset        | [std\_msgs/Empty](http://docs.ros.org/en/api/std_msgs/html/msg/Empty.html) | Empty messages sent to reset the navigation filter |
| /#vehicle#/nav/filter/state        | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | Filtered state of the vehicle |
| /tf                               | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | Transform ROS message to switch between reference frames (rotations) |
| /tf\_static                       | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | Static reference frame |

## Publishers
| Publishers                     | msgs type                            | Purpose                                                      |
| -----------                    | --------------                       | ---------                                                    |
| /#vehicle#/State\_dr            | [farol\_msgs/mState](https://github.com/dsor-isr/farol/blob/main/farol_msgs/msg/mState.msg) | Vehicle state determined using dead reckoning (sensors other than GPS and USBL, such as DVL, IMU, etc.) in message form to send to the console |
| /#vehicle#/nav/filter/currents  | [farol\_msgs/Currents](https://github.com/dsor-isr/farol/blob/main/farol_msgs/msg/Currents.msg) | Current estimation |
| /#vehicle#/nav/filter/state     | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | Filtered state of the vehicle |
| /#vehicle#/nav/filter/state\_dr | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | Vehicle state determined using dead reckoning (sensors other than GPS and USBL, such as DVL, IMU, etc.) |
| /tf                            | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | Transform ROS message to switch between reference frames (rotations) |

## Services
| Services                         | msgs type                                                                      | Purpose                                     |
| -----------                      | --------------                                                                 | ---------                                   |
| /#vehicle#/nav/reset\_filter\_dr | [std\_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html) | Service to reset dead reckoning filter estimation |

## Parameters
| Parameters                                                              | type   | Default     | Purpose                       |
| -----------                                                             | ----   | ----------  | ---------                     |
| /#vehicle#/nav/filter/dvl/body\_frame                                   | bool   | true        | Option to incorporate body frame definition for DVL sensor |
| /#vehicle#/nav/filter/kalman\_filter/bypass\_ahrs                       | bool   | false       | Option to bypass AHRS measurements |
| /#vehicle#/nav/filter/kalman\_filter/manually\_initialization/frame\_id | string | gnss        | Option on how to initialize the navigation filter |
| /#vehicle#/nav/filter/kalman\_filter/manually\_initialization/noise     | array  | [1.0, 1.0, 0.01, 0.001, 0.001, 0.01, 0.02, 0.03, 0.04, 0.01, 0.01, 0.01, 0.0, 0.0, 0.01] | Sensor noise for the filter |
| /#vehicle#/nav/filter/kalman\_filter/manually\_initialization/value     | array  | [4290794.432828665, 491936.5610790758, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0.0, 0.0, 0.0, 0.0, 5.0] | Initial values for the filter |
| /#vehicle#/nav/filter/kalman\_filter/outlier\_rejection\_threshold      | int    | 15 | Number of outlier values to reject before accepting them |
| /#vehicle#/nav/filter/kalman\_filter/predict\_period                    | float  | 0.1 | Time period between predictions |
| /#vehicle#/nav/filter/kalman\_filter/process\_covariance                | array  | [1.0, 0.1, 0.1, 0.1, 0.1, 0.1] | Process noise covariance for the filter |
| /#vehicle#/nav/filter/kalman\_filter/reject\_counter                    | array  | [8, 200, 12, 4, 7, 6] | |
| /#vehicle#/nav/filter/kalman\_filter/reset\_period                      | int    | 0 | outlier reject counters for position, velocity, angle, angle rate, acceleration, altitude |
| /#vehicle#/nav/filter/kalman\_filter/save\_measurement\_interval        | int    | 5 | Buffer for saving measurements while usbl fix is not received |
| /#vehicle#/nav/filter/kalman\_filter/sensors                            | -      | - | Sensor configurations for navigation deployment (see `nav.yaml` for more details) |
| /#vehicle#/nav/filter/name\_vehicle\_id                                 | string | /#vehicle#/ | Vehicle name (type) and ID |
| /#vehicle#/nav/filter/node\_frequency                                   | float  | 10 | Working frequency of node |
| /#vehicle#/nav/filter/originLat                                         | float  | 38.765852 | Origin Latitude for inertial frame |
| /#vehicle#/nav/filter/originLon                                         | float  | -9.09281873 | Origin Longitude for inertial frame |
| /#vehicle#/nav/filter/tf/broadcast                                      | bool   | true | Flag to publish transform to rotate frames |
| /#vehicle#/nav/filter/tf/frames/base\_link                              | string | base\_link | Type of frame |
| /#vehicle#/nav/filter/tf/frames/map                                     | string | map | Type of map for world that is being loaded |
| /#vehicle#/nav/filter/tf/frames/odom                                    | string | null | Odometry related parameter |
| /#vehicle#/nav/filter/tf/frames/world                                   | string | map | Type of world that is being loaded |