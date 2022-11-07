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
| /mvector0/Flag                    | [std\_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html) | |
| /mvector0/measurement/orientation | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | |
| /mvector0/measurement/position    | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | |
| /mvector0/measurement/velocity    | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/main/dsor_msgs/msg/Measurement.msg) | |
| /mvector0/nav/filter/reset        | [std\_msgs/Empty](http://docs.ros.org/en/api/std_msgs/html/msg/Empty.html) | |
| /mvector0/nav/filter/state        | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | |
| /tf                               | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | |
| /tf\_static                       | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | |

## Publishers
| Publishers                     | msgs type                            | Purpose                                                      |
| -----------                    | --------------                       | ---------                                                    |
| /mvector0/State\_dr            | [farol\_msgs/mState](https://github.com/dsor-isr/farol/blob/main/farol_msgs/msg/mState.msg) | |
| /mvector0/nav/filter/currents  | [farol\_msgs/Currents](https://github.com/dsor-isr/farol/blob/main/farol_msgs/msg/Currents.msg) | |
| /mvector0/nav/filter/state     | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | |
| /mvector0/nav/filter/state\_dr | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | |
| /tf                            | [tf2\_msgs/TFMessage](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) | |

## Services
| Services                        | msgs type                           | Purpose                                     |
| -----------                     | --------------                      | ---------                                   |
| /mvector0/nav/reset\_filter\_dr | [std\_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html) |                                             |

## Parameters
| Parameters                 | type   | Default     | Purpose                       |
| -----------                | ----   | ----------  | ---------                     |
| /mvector0/nav/filter/dvl/body_frame | bool | true | |
| /mvector0/nav/filter/kalman_filter/bypass_ahrs | bool | false | |
| /mvector0/nav/filter/kalman_filter/manually_initialization/frame_id | string | gnss | |
| /mvector0/nav/filter/kalman_filter/manually_initialization/noise
| /mvector0/nav/filter/kalman_filter/manually_initialization/value
| /mvector0/nav/filter/kalman_filter/outlier_rejection_threshold
| /mvector0/nav/filter/kalman_filter/predict_period
| /mvector0/nav/filter/kalman_filter/process_covariance
| /mvector0/nav/filter/kalman_filter/reject_counter
| /mvector0/nav/filter/kalman_filter/reset_period
| /mvector0/nav/filter/kalman_filter/save_measurement_interval
| /mvector0/nav/filter/kalman_filter/sensors
| /mvector0/nav/filter/name_vehicle_id
| /mvector0/nav/filter/node_frequency
| /mvector0/nav/filter/originLat
| /mvector0/nav/filter/originLon
| /mvector0/nav/filter/services_dr/reset_filter_dr
| /mvector0/nav/filter/tf/broadcast
| /mvector0/nav/filter/tf/frames/base_link
| /mvector0/nav/filter/tf/frames/map
| /mvector0/nav/filter/tf/frames/odom
| /mvector0/nav/filter/tf/frames/world