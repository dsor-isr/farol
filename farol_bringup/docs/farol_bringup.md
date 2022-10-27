# farol_bringup Node

## In a nutshell
This node is launched by the custom bringup and subsequently launches the whole system, using the default configurations inside the package `farol_bringup`, and later loading the new configurations to specifically override certain already launched variables.

## Diagram
![farol_bringup Diagram](img/farol_bringup.png)

## Subscribers
| Subscribers         | msgs type                                                                        | Purpose                      |
| -----------         | --------------                                                                   | ---------                    |
| /#vehicle/sensors/* | [std_msgs](http://docs.ros.org/en/api/std_msgs/html/index-msg.html)              | information from the sensors |
| /#vehicle/State     | [farol_msgs/mState](https://dsor-isr.github.io/farol/farol-ros-messages/mState/) | State of the vehicle         |
|                     |                                                                                  |                              |

## Publishers
| Publishers          | msgs type                                                                   | Purpose                                  |
| -----------         | --------------                                                              | ---------                                |
| /#vehicle/sensors/* | [std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html) | Mission string to be done by the vehicle |

## Services
* None

## Parameters
| Parameters                                      | type   | Default | Purpose                                                   |
| ----------                                      | ----   | ------- | -------                                                   |
| /#vehicle#/addons/console_server/PORT           | int    | 7080    | TCP port                                                  |
| /#vehicle#/addons/console_server/ROOT_NAMESPACE | bool   | True    | Use private namespace                                     |
| /#vehicle#/addons/console_server/pages_folder   | string | ./      | Folder wit vehicle webpages                               |
| /#vehicle#/addons/console_server/Mission_folter | string | -       | Folder with stored txt files with path following missions |
