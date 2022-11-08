# remote_controller Node

## In a nutshell
Remote controller ROS node class. Receives from a joystick (using pygame) the desired controls for the inner-loops and publishes these periodically (at a predefined frequency) to the inner-loops of the vehicle.

## Diagram

![remote\_controller Diagram](img/remote_controller.png)

## Subscribers

| Subscribers                 | msgs type                                                    | Purpose                     |
| --------------------------- | ------------------------------------------------------------ | --------------------------- |
| /#vehicle#/nav/filter/state | [auv\_msgs/NavigationStatus](http://docs.ros.org/en/api/auv_msgs/html/msg/NavigationStatus.html) | State of the robot platform |

## Publishers

| Publishers              | msgs type                                                    | Purpose                                                      |
| ----------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| /#vehicle#/ref/surge    | [std\_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) | surge ref to inner loop control                              |
| /#vehicle#/ref/sway     | [std\_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) | sway ref to inner loop control                               |
| /#vehicle#/ref/heave    | [std\_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) | heave ref to inner loop control                              |
| /#vehicle#/ref/yaw\_rate | [std\_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) | yaw\_rate ref to inner loop control                           |
| /#vehicle#/ref/depth    | [std\_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) | depth ref to inner loop control                              |
| /#vehicle#/ref/yaw      | [std\_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) | yaw ref to inner loop control, not sent via remote controller, but using last kown yaw from robot state. Published when yaw\_rate is not active. |

## Services
* None

## Parameters
| Parameters | type   | Default   | Purpose                                                      |
| ---------- | ------ | --------- | ------------------------------------------------------------ |
| joystick   | string | gamesir   | Name of the joystick being used. **Note:** It has its own configurations, see available examples and use the following [webpage](https://gamepad-tester.com/), for getting the right buttons ids, when configuring a new remote controller. |
| mode       | string | bluetooth | Configurations for both usb and bluetooth                    |
