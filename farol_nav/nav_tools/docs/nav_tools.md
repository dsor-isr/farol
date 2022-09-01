# Nav tools

## Project Description

This package contains tools to convert msgs between data types. Currently, it also houses range-based positioning tools. This package has the following highlights

- Convert data in WGS84 from *sensor_msgs::NavSatFix* to data in UTM NED & *farol_msgs::Measurement*
- Convert state from *auv_msgs::NavigationStatus* to *farol_msgs::mState*
- Convert state from *auv_msgs::NavigationStatus* to *nav_msgs::Odometry*
- Convert Ranges from single-beacon to position update using EKF
- Convert Georeferenced USBL position fix to position update

## Getting Started

### Installation

Pkg `nav_tools` comes as part of the farol_vx stack.

## Gnss2Utm

This node converts data in WGS84 from *sensor_msgs::NavSatFix* to UTM NED *farol_msgs::Measurement* format

### Configuration

This section explains how to write the node configuration file. A sample is given below

```yaml
node_frequency: 10
topics:
  subscribers: [ "/measurement/gnss" ]
  publishers: [ "/measurement/position" ]
```

**ROS Node Parameters**

1. **node_frequency**: parameter of type *double*
   - Define the node output frequency
2. **topics/subscribers**: parameter of type *string[1]*
   - Defines the input topic to receive position in Lat/Lon, WGS84 format.
3. **topics/publishers**: parameters of type *string[1]*
   - Defines topic to output position in UTM NED

### Launching the Nodes

An example launch file is provided below where `nav_tools/gnss2utm` node is being launched.

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="nav_tools" type="gnss_to_utm" name="gnss2utm" respawn="true" output="screen">
        <rosparam command="load" file="$(find nav_tools)/config/gnss2utm.yaml"/>
    </node>
</launch>
```

## AuvState2mState

This node converts state in *auv_msgs::NavigationStatus* to state in *farol_msgs::mState*. Also need inside_pressure data for mState.

### Configuration

This section explains how to write the node configuration file. A sample is given below

```yaml
node_frequency: 10
topics:
  subscribers: [ "/nav/filter/state", "/drivers/inside_pressure/data" ]  
  publishers:  [ "/State" ] 
```

1. **node_frequency**: node output frequency

2. **topics/subscribers:** parameter of type *string[2]*
- topic[1] - state: receives state of type auv_msgs::NavigationStatu
   - topic[2] - inside_pressure: receives inside_pressure data

3. **topics/publishers:** parameter of type *string[1]*
- topic[1] - state: publishes state of type farol_msgs::mState

### Launching the Nodes

An example launch file is provided below where `nav_tools/auvstate2mstate` node is being launched.

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="nav_tools" type="auvstate_to_mstate" name="auvstate2mstate" respawn="true" output="screen">
        <rosparam command="load" file="$(find nav_tools)/config/auvstate2mstate.yaml"/>
    </node>
</launch>
```

## AuvState2Odom

This node converts state in *auv_msgs::NavigationStatus* to *nav_msgs::Odometry*.

### Configuration

This section explains how to write the node configuration file. A sample is given below

```yaml
node_frequency: 10
topics:
  subscribers: [ "/nav_filter/state" ]
  publishers:  [ "/nav_filter/state_odom" ]
```

1. **node_frequency**: node output frequency

2. **topics/subscribers:** parameter of type *string[1]*
- receives state of type auv_msgs::NavigationStatus
   
3. **topics/publishers:** parameter of type *string[1]*
- publishes state of type nav_msgs::Odometry

### Launching the Nodes

An example launch file is provided below where `nav_tools/auvstate2odom` node is being launched.

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="nav_tools" type="auvstate_to_odom" name="auvstate2odom" respawn="true" output="screen">
        <rosparam command="load" file="$(find nav_tools)/config/auvstate2odom.yaml"/>
    </node>
</launch>
```

## Usbl2Pos

This node takes in georeferences usbl position fix and converts it to position updates of the filter. It can be deployed in two ways

**Scenario 1: Inverted-USBL** In this scenario, the underwater vehicle localizes itself with respect to an anchor, whose precise global position is known. The usbl onboard the vehicle receives the georeferenced position of the anchor and the usbl-fix. The vehicle uses these two information to estimate its position using simple geometry.

**Scenario 2: Tracking with USBL** In this scenario, the anchor, whose precise global position is known, tracks underwater vehicles using USBL fixes. The anchor receives the usbl-fix of the underwater vehicle and uses its own position to estimate the position of the underwater vehicle.

### Configuration

```yaml
node_frequency: 10
topics:
  subscribers: [
    "/acomms/measurement/usbl_fix", "/acomms/convert/state"
  ]
  publishers: [ "/measurement/position" ]
t_sync: 2
fix_type: false
meas_noise: 0.001
```

1. **node_frequency**: node output frequency

2. **topics/subscribers:** parameter of type *string[2]*
- defines the input topics -
   - usblfix topic to receive USBL position fixes in *farol_msgs::mUSBLFix*
- state topic to receive the state of the vehicle or anchor state in *auv_msgs::NavigationStatus*
   
3. **t_sync:** parameter of type *double*
- time (secs) after which "Range Only" and "Bearing/Elevation" *farol_msgs::mUSBLFix* fixes go out of sync
   
4. **fix_type:** parameter of type *bool*
- if set false, vehicle acts as an achor, estimates other vehicles position i.e. scenario 1
   - if set true, vehicle estimates its position with respect to an anchor i.e. scenario 2

### Launching the Nodes

An example launch file is provided below where `nav_tools/usblfix2pos` node is being launched.

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="nav_tools" type="usblfix_to_pos" name="usblfix2pos" respawn="true" output="screen">
        <rosparam command="load" file="$(find nav_tools)/config/usblfix2pos.yaml"/>
    </node>
</launch>
```

## Range2Pos

This node converts ranges from a single beacon to a position measurement using an Extended Kalman Filter.

### Configuration

This section explains how to write the node configuration file. A sample is given below

```yaml
node_frequency: 10
t_sync: 2
topics:
    subscribers: [ "/sensors/usbl", "/nav/filter/state" ]
    publishers: [ "/measurement/position" ]
beacon:
    position: [4290771, 491886]
```

1. **node_frequency**: node output frequency

2. **topics/subscribers:** parameter of type *string[3]*
- Defines the input topics -
   - range topic to receive range only USBL fixes in *farol_msgs::mUSBLFix*
- state topic to receive state from navigation system in *auv_msgs::NavigationStatus*
   - beacon topic to receive beacon state

3. **topics/publishers:** parameter of type *string[1]*
- publishes position update of type farol_msgs::Measurement
   
4. **beacon/position**: State of stationary beacon position in UTM NED

### Launching the Nodes

An example launch file is provided below where `nav_tools/auvstate2mstate` node is being launched.

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="nav_tools" type="usblfix_to_pos" name="range2pos" respawn="true" output="screen">
        <rosparam command="load" file="$(find nav_tools)/config/range2pos.yaml"/>
    </node>
</launch>
```