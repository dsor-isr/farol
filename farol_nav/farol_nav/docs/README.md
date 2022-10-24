## Description

`farol_nav` is a meta-package to house all the software packages required to construct a navigation system for an unmanned underwater vehicle. This package has the following features

- Deploy vehicle state estimation node for dead-reckoning
- Deploy vehicle state estimation node with position fixes from one or multiple sources
- Implement Single-beacon and USBL range based navigation algorithm
- Convert WGS84 input in to UTM NED in
- Convert State in to State in
- Convert State in to State in
- Implements tools and techniques used in geophysical navigation.

## Getting Started

### Prerequisite Information

Before getting started, the reader is recommended to get familiarized with the following concepts - Kalman and Extended Kalman Filter - Range based navigation techniques such as Single-beacon (EKF) and USBL - Reference Frames and Transformations - ROS and Linux operations - Key Libraries include TF2, Geographic Lib, Eigen

### Conventions

Assume that the best practices laid down by Péter Fankhauser, ANYbotics are followed unless other stated. Deviations - AUV World reference frame is North-East-Down - Input Angles are in Radians, Output Angles are in Degrees - Following coordinate frames are used - base_link: Body-fixed rigid frame attached to the COM of the vehicle - odom: World-fixed frame where position evolves smoothly, without discrete jumps but with drifts. - map : World-fixed frame where position evolves with discrete jumps, but with little drifts.

### Inspirations

Work done in this package draws heavily from the following two packages - farol-ros/farol_control/filters_farol - cra-ros-pkg/robot_localization

! Follow the Guidelines and Development Lifecycle defined for farol_vx stack

## Project Directory

### sensor_fusion

A general-purpose kalman filter for vehicle state estimation. For documentation, refer the links below - Readme.md - Developer Notes - Theory

### nav_tools

Contains handy tools used to convert measurements and state messages. Also contains range-based measurement nodes.

### farol_gn

Geo-physical navigation