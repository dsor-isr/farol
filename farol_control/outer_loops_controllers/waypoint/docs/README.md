# Waypoint Package

This package takes into account different waypoint functionalities for the vehicle's outer loop. Desired references are calculated so that the vehicle converges to the specified waypoint. The topic *Flag* is also closely monitored, since it dictates the general state of the vehicle (more on this below).

## Nodes
* [waypoint](waypoint.md)

## Services
* [sendWpType1](sendWpType1.md)

## Dependencies
* None

## The *Flag* Topic

The very global topic *Flag* ([std_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html)) is used to specify the state in which the vehicle finds itself. In other words, the *Flag* topic determines if the vehicle should be idle, converging to a waypoint or following a path. It can be even redesigned to accommodate more states (custom) than the already defined in the table below:

| Flag | State |
| --- | --- |
| 0 | Idle |
| 4 | Waypoint |
| 6 | Path Following |