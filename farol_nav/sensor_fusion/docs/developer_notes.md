# Developer Notes

## Project Directory

```
.
├── CMakeLists.txt
├── config
│   ├── map.yaml
│   └── odom.yaml
├── docs
│   ├── figs
│   ├── readme.md
│   ├── theory.md
│   └── developernotes.md
├── include
│   ├── algorithms
│   │   ├── HorizontalFilter.h
│   │   ├── RotationalFilter.h
│   │   └── VerticalFilter.h
│   └── ros
│       └── FiltersNode.h
├── launch
│   └── sensor_fusion.launch
├── package.xml
├── scripts
│   └── filtersScript
├── src
│   ├── algorithms
│   │   ├── HorizontalFilter.cpp
│   │   ├── RotationalFilter.cpp
│   │   └── VerticalFilter.cpp
│   └── ros
│       └── FiltersNode.cpp
└── test
    └── filters_test.cpp
```

Pkg *sensor_fusion* follows the code methodology adapated for the farol_vx stack and hence seperates the code into algorithm and node class. The "algorithm" section implements the kalman filter based sensor fusion and the "node" section implements all things required for integration with ROS.

1. **config**: contains sample config files for and filters
2. **docs** : contains documentation for the pkg
3. **include**: contains header files class and node class
4. **launch**: contains sample launch files
5. **src**: contains the main code for algorithm and node class

Pkg contains no scripts or tests yet.

## Algorithm Class

This class implements a generic kalman based sensor fusion system which fuses input from an arbitary number of sources to estimate the state of a vehicle. To make things simpler, the kalman filter is divided into three seperate filters which estimate the horizontal, vertical and rotational state respectively.

1. HorizontalFilter.h & HorizontalFilter.cpp
   - Time-delayed Kalman-Filter to estimate position (x, y), velocity (vx, vy), acceleration (ax, ay) and currents (cx, cy)
   - Dynamic model: Constant Acceleration
   - `measCallback()`: Receives measurements in the horizontal plane
   - `addMeasurement()`: Adds valid measurements to a measurement list
   - `forwardPropagation()`: Estimates and updates the state vector from the point of time when the measurement was received.
   - `predict()`: Predict Stage of the Kalman Filter
   - `update()`: Update Stage of the Kalman Filter
   - `configure()`: `public method` Configure the filter using loaded parameters
   - `getEstimate()`: `public method` Returns the latest state estimate
2. VerticalFilter.h & VerticalFilter.cpp
   - Kalman-Filter to estimate position (z), Altitude, velocity (vz) and Bouyancy (B)
   - Dynamic model: ?
3. RotationalFilter.h & RotationalFilter.cpp
   - Kalman-filter to estimate rotation (r, p, h) and rotation rate (vr, vp, vh)
   - Measurements can be treated as inputs which bypasses the whole filter

Estimates from individual filters are fused inside the node class to produce the net result.

## ROS Class

This class implements ROS functionalities that are

1. *Parameters*: Used to configure the three filters
2. *Publishers*:
   - `state_pub`: Publishes state estimate
3. *Subscribers*:
   - `measCallback(farol_msgs::Measurements)`: Multiple topics use a single generic callback which processes, transforms and distributes the input to the three algorithm classes.
4. *Timers*:
   - `stateTimerCallback()`: Gets the latest state estimate from all three algorithm class
   - `listTimerCallback()`: Clears the previous measurements from the list in HorizontalFilter