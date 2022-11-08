# gnss2utmoutlier Node

## In a nutshell
This node converts data in WGS84 from *sensor_msgs::NavSatFix* to UTM NED *farol_msgs::Measurement* format, just like the *gnss2utm* node, but creating gps outliers at the same time.

## Diagram
![Gnss2UtmOutlier Diagram](img/gnss2utmoutlier.png)

## Subscribers
| Subscribers            | msgs type                                                                                | Purpose                               |
| -----------            | --------------                                                                           | ---------                             |
| /#vehicle#/sensor/gnss | [sensor\_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) | GNSS information from mounted sensors |

## Publishers
| Publishers                       | msgs type                  | Purpose                                                                    |
| -----------                      | --------------             | ---------                                                                  |
| /#vehicle#/measurement/positions | [dsor\_msgs/Measurement](https://github.com/dsor-isr/dsor_utils/blob/d39195370a36517fc0c1a05c8e043f58720416bb/dsor_msgs/msg/Measurement.msg) | Measurement message of vehicles position estimated acquired by the sensors |

## Services
| Services                                    | msgs type                                                                      | Purpose                                                   |
| -----------                                 | --------------                                                                 | ---------                                                 |
| /#vehicle#/sensor/fake/enable\_gps\_outlier | [std\_srvs/SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html) | Service to enable the gps outlier creation functionality  |
| /#vehicle#/sensor/fake/enable\_usbl\_delay  | [std\_srvs/SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html) | Service to enable delay while using acoustic usbl sensors |

## Parameters
| Parameters                 | type   | Default     | Purpose                       |
| -----------                | ----   | ----------  | ---------                     |
| /#vehicle#/node\_frequency | float  | 10.0        | Working frequency of the node |
