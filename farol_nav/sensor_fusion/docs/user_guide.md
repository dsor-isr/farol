# User-Guide

## Getting Started

### Installation
Pkg `sensor_fusion` comes as part of the farol_vx stack.

### Configuration
This section explains how to write the node configuration file. A sample is given below

#### ROS Node Parameters

```yaml
node_frequency: 10                                              # ROS node output rate
topics:
    subscribers: [                  # std::string containing topics < reset, position, velocity, orientation> 
        "reset", "/measurement/position", "/measurement/velocity", "/measurement/orientation"
    ]
    publishers: [                                               # std::string containing topics <state, debug>
        "state", "debug"
    ]
```

#### Class Node Parameters

```
tf:                                                             # TF publishers
    broadcast: true                                             # flag to publish node TF frames
    frames: [                                                   # std::string containing frame <base_link, odom, map, world>
        "base_link", "odom", "map", "map"
    ]
kalman_filter:
    config:             [0.1, 5.0, 16]                          # predict period, save measurement interval, outlier rejection threshold 
    reject_counter:     [8, 5, 12, 3, 3, 3]                     # outlier reject counters for position, velocity, angle, angle rate, acceleration, altitude
    process_covariance: [0.03, 0.015, 0.85, 1.2, 0.9, 0.12]     # process noise for position, velocity, angles, angle rate, acceleration, altitude
    vertical_drag:      [-0.1287, -0.4097, -0.013]              # alpha, beta, bouyancy
    bypass_ahrs: false                                          # Set True to treat AHRS as input
    initialize:                                                 # Define initialization condition
        trigger: false                                          # Set true to initialize at start
        meas:                                                   # If trigger is set to true, define initial position and state. If it is set to false, define initial state except position
            frame_id: "gnss"                                    # If trigger is set to false, define measurement frame used for initialization
            value: [0, 0, 0,                                    # Position - x, y, z             
                    0, 0, 0,                                    # Velocity - vx, vy, vz
                    0, 0, 0,                                    # Orientation - r, p, h
                    0, 0, 0,                                    # Orientation Rate - dr, dp, vh
                    0, 0, 0]                                    # Acceleration & Altitude - ax, ay, A
            noise: [1000, 1000, 1000,                           # If trigger is set to true, define initial position and state covariance. 
                    0.1 , 0.1 , 0.1 ,                           # If trigger is set to false, define initial state covariance except position
                    1   , 1   , 1   ,                           # Array format is same as above
                    0.1 , 0.1 , 0.1 ,
                    0.01, 0.01, 1000]
    sensors:                                                    # Define sensors as input here. Any sensor frame ids not defined here will be ignored by the filter     
        -   frame_id:   "gnss"                                  # Example Sensor 1: Sat Nav Input 
            config:     [true,  true,  false,                   # frame_id takes in the frame_id associated with the input measurement
                        false, false, false,                    # Config is a 15-state bool array that defines the input measurement 
                        false, false, false,                    # The number of inputs should be equal to the no of "true" flags in the config array
                        false, false, false,                    # For sensor providing position (x, y) input, config has two flags triggered as shown
                        false, false, false]                    # 
            noise:      [0.5, 0.5, 0.0,                         # Noise if an optional 15-state double array that overrides incoming measurement noise 
                        0.0, 0.0, 0.0,                          # Filter will always take 0.5 as measurement noise during update stage of the kalman filter here
                        0.0, 0.0, 0.0,                          # 
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0]
        -   frame_id:   "dvl_bt"                                # Example Sensor 2: DVL Bottom Track
            config:     [false, false, false,                   # Sensor updates inertial position of the vehicle
                        true,  true,  false,                    # For sensor providing velocity (vx, vy) input, config has two flags triggered as shown
                        false, false, false,                    # Filter takes in noise measurement from the input, ignore the input if there is none
                        false, false, false,
                        false, false, false]
        -   frame_id:   "depth"                                 # Example Sensor 3: Depth Sensor
            config:     [false, false, true,                    # Sensor updates depth of the vehicle
                        false, false, false,                    # For sensor providing Depth (z) input, config has one flag triggered as shown
                        false, false, false, 
                        false, false, false, 
                        false, false, false]
            noise:      [0.0, 0.0, 0.0, 
                        0.0, 0.0, 1.0, 
                        0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0]
        -   frame_id:   "ahrs"                                  # Example Sensor 4: AHRS Sensor
            config:     [false, false, false,                   # Sensor updates inertial position of the vehicle
                        false, false, false,                    # For sensor providing rotation (r, p, h, vr, vp, vh) input, config has 6 flags triggered as shown
                        true,  true,  true,  
                        true,  true,  true,  
                        false, false, false]
            noise:      [0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 
                        1.0, 1.0, 1.0, 
                        1.0, 1.0, 1.0, 
                        0.0, 0.0, 0.0]
```

### Config Breakdown

#### ROS Node Parameters**

| Name               | Type      | Function                                                   |
|--------------------|-----------|------------------------------------------------------------|
| node_frequency     | int       | Filter output frequency                                    |
| topics/subscribers | string[4] | Input topics for reset, position, velocity and orientation |
| topics/publishers  | string[2] | Output topics for state estimate and debug info            |

*Remarks*
- position, velocity, orientation topics subscribe to input of type `farol_msgs::Measurement`
- state topic publishes state estimate in `auv_msgs::NavigationStatus`
- topic debug publishes debug information in `std::string`

#### Class Parameters

**TF**

| Name         | Type      | Function                                              |
|--------------|-----------|-------------------------------------------------------|
| tf/broadcast | bool      | Flag to trigger state broadcast to ros::tfs           |
| tf/frames    | string[4] | Frame names from base_link, odom, map and world frame |

*Remarks*
- If dead reckoning is not needed, set odom frame as "null"
- If tf/broadcast flag is set to true while `odom` frame is not set, filter will publish the tf tree as: `base_link` -> `map` -> `world` 
- If tf/broadcast flag is set to true while `odom` frame is set, filter will publish the tf tree as: `base_link` -> `odom` -> `map` -> `world` 

**Kalman Parameters**

| Name                             | Type      | Function                                                                                         |
|----------------------------------|-----------|--------------------------------------------------------------------------------------------------|
| kalman_filter/config             | double[3] | Set parameters for predict period, save measurement interval, outlier rejection threshold        |
| kalman_filter/reject_counter     | int[6]    | Set outlier reject counters for position, velocity, angle, angle rate, acceleration and altitude |
| kalman_filter/process_covariance | double[6] | Set process noise for position, velocity, angles, angle rate, acceleration and altitude          |
| kalman_filter/vertical_drag      | double[3] | Set alpha, beta, bouyancy for vertical filter                                                    |
| kalman_filter/bypass_ahrs        | bool      | Trigger to treat AHRS as an Input                                                                |

*Remarks*
- `predict period` defines the frequency of kalman filter predicts stage
- `save measurement interval` sets the interval over which measurements are rejected by the filter. If no input is received for more than twice this interval, filter resets.
- `outlier rejection threshold` sets the threshold for outlier measurement rejection.
- `reject counter` defines the no of measurements to reject before accepting outliers as the input

**Initialization Parameters**


- `` Defines the initial covariance of the state vector
- Defines the drag parameters used in the state model of vertical filter.
- Flag to treat inputs to the rotation filter as input. If set to true, any input sent to the node bypasses the filter and is output directly.

**Initialization Parameters**

| Name                       | Type       | Function                                  |
|----------------------------|------------|-------------------------------------------|
| kalman_filter/initialize/. |            |                                           |
| ./trigger                  | bool       | Flag to trigger initialization at startup |
| ./meas/frame_id            | string     | Sensor frame to initialize filter with    |
| ./meas/value               | double[15] | Defines the initial state of the vessel   |
| ./meas/noise               | bool[15]   | Defines the initial state covariance      |

*Remarks* 
- If `trigger` is set to false, filter will initialize horizontal position with a measurement with frame `./meas/frame_id`. The rest is set by the `./meas`

**Sensor Input** 
Input sensors to the filter are described as a list of type `<XmlRpc::XmlRpcValue>`. Any number of sensors may be defined following the format described below.

| Name                      | Type       | Function                                            |
|---------------------------|------------|-----------------------------------------------------|
| kalman_filter/sensors/.   |            |                                                     |
| - frame_id, config, noise |            |                                                     |
| frame_id                  | string     | Defines the name of the sensor frame.               |
| config                    | bool[15]   | Defines the observation matrix of the kalman filter |
| noise (optional)          | double[15] | Defines the noise associated with the measurement.  |

*Remarks*
- Each sensor to the filter must have a unique frame_id and a static TF to base_link associated with it.
- Incoming measurement must always contain the same no of inputs, as described in the observation matrix.
- If the `noise` in input measurement is zero or invalid, filter will ignore the measurement.

## Launching the Nodes
An example launch file is provided below where two sensor_fusion/sensor_fusion nodes are being launched, named filter and filter_dr.

```xml
<?xml version="1.0"?>
<launch>

<env name="ROSCONSOLE_FORMAT" value="[${severity}] [${time}]: ${node}: ${message}"/>

<!-- ######################## -->
<!-- Parameters and Arguments -->
<!-- ######################## -->
    <arg name="map"        default="map"   />  <!-- name of config file for node ~filter    -->
    <arg name="odom"       default="odom"  />  <!-- name of config file for node ~filter_dr -->

    <group ns="nav">
        <!-- ############################ -->
        <!-- filters_dr: Estimates the position of the vehicle in <odom> frame. -->
        <!-- ############################ -->
        <node pkg="sensor_fusion" type="sensor_fusion" name="filter_dr" respawn="false">
            <rosparam command="load" file="$(find farol_bringup)/config/$(arg odom).yaml" />
        </node>

        <!-- ############################ -->
        <!-- filters: Estimates the position of the vehicle in <map> frame. -->
        <!-- ############################ -->
        <node pkg="sensor_fusion" type="sensor_fusion" name="filter" respawn="false" output="screen">
            <rosparam command="load" file="$(find farol_bringup)/config/$(arg map).yaml" />
        </node>
    </group>
</launch>
```