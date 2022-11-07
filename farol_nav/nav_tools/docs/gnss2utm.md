# gnss2utm Node

## In a nutshell
This node converts data in WGS84 from *sensor_msgs::NavSatFix* to UTM NED *farol_msgs::Measurement* format.

## Diagram
![Gnss2Utm Diagram](img/gnss2utm.png)

## Subscribers
| Subscribers            | msgs type                                                                                | Purpose                               |
| -----------            | --------------                                                                           | ---------                             |
| /#vehicle#/sensor/gnss | [sensor\_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) | GNSS information from mounted sensors |

## Publishers
| Publishers                       | msgs type                                                                         | Purpose                                                                    |
| -----------                      | --------------                                                                    | ---------                                                                  |
| /#vehicle#/State\_gt             | [farol\_msgs/mState](https://dsor-isr.github.io/farol/farol-ros-messages/mState/) | Ground truth state of the vehicle                                          |
| /#vehicle#/measurement/positions | [dsor\_msgs/Measurement]()                                                        | Measurement message of vehicles position estimated acquired by the sensors |

## Services
| Services                         | msgs type                                                                      | Purpose                                 |
| -----------                      | --------------                                                                 | ---------                               |
| /#vehicle#/sensor/enable\_gps    | [std\_srvs/SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html) | Service to enable the gps functionality |

## Parameters
| Parameters                 | type   | Default     | Purpose                       |
| -----------                | ----   | ----------  | ---------                     |
| /#vehicle#/node\_frequency | float  | 10.0        | Working frequency of the node |
