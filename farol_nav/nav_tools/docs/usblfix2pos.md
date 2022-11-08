# usbl2pos Node

## In a nutshell
This node takes in georeferences usbl position fix and converts it to position updates of the filter. It can be deployed in two ways:
* **Scenario 1: Inverted-USBL** In this scenario, the underwater vehicle localizes itself with respect to an anchor, whose precise global position is known. The usbl onboard the vehicle receives the georeferenced position of the anchor and the usbl-fix. The vehicle uses these two information to estimate its position using simple geometry.
* **Scenario 2: Tracking with USBL** In this scenario, the anchor, whose precise global position is known, tracks underwater vehicles using USBL fixes. The anchor receives the usbl-fix of the underwater vehicle and uses its own position to estimate the position of the underwater vehicle.

## Diagram
![UsblFix2Pos](img/usbl2pos.png)

## Subscribers
| Subscribers                        | msgs type                                                                                                                                       | Purpose                                                 |
| -----------                        | --------------                                                                                                                                  | ---------                                               |
| /#vehicle#/sensors/usbl\_fix       | [farol\_msgs/mUSBLFix](https://dsor-isr.github.io/farol/farol-ros-messages/mUSBLFix/)                                                            | USBL estimation from the sensors                        |
| /#vehicle#/acomms/nav/filter/state | [auv\_msgs/NavigationStatus](https://github.com/oceansystemslab/auv_msgs/blob/1faaddd7ee6e9c2c9869e3d8dcff92bb56c2fce4/msg/NavigationStatus.msg) | Filtered state received through acoustic communications |

## Publishers
| Publishers                       | msgs type                  | Purpose                                                                    |
| -----------                      | --------------             | ---------                                                                  |
| /#vehicle#/measurement/positions | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/d39195370a36517fc0c1a05c8e043f58720416bb/dsor_msgs/msg/Measurement.msg) | Measurement message of vehicles position estimated acquired by the sensors |
| /#vehicle#/State\_usbl\_est      | [farol\_msgs/mState](https://dsor-isr.github.io/farol/farol-ros-messages/mState/)     | State estimated from USBL measurements |

## Services
| Services                         | msgs type                                                                      | Purpose                                 |
| -----------                      | --------------                                                                 | ---------                               |
| /#vehicle#/sensor/fake/enable\_gps\_outlier | [std\_srvs/SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html) | Service to enable the gps outlier creation functionality |
| /#vehicle#/sensor/fake/enable\_usbl\_delay  | [std\_srvs/SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html) | Service to enable delay while using acoustic usbl sensors |

## Parameters
| Parameters                 | type   | Default     | Purpose                       |
| -----------                | ----   | ----------  | ---------                     |
| /#vehicle#/node\_frequency | float  | 10.0        | Working frequency of the node |
| /#vehicle#/fix\_type       | bool   | false       | Type USBL fix to apply        |
| /#vehicle#/meas\_noise     | float  | 0.001       | Noise to apply                |
| /#vehicle#/t\_sync         | float  | 2           | Synchronization time          |
