# Solo Package - console_path_parser

## Run
```
roslaunch console_path_parser console_path_parser.launch
```

## Launch file
**launch/console_path_parser.launch**
```xml
<?xml version="1.0"?>
<launch>

 <!-- small description about your node -->

 <!--<node pkg="my_package_name" type="my_node_name" name="my_node_name"
 respawn="false" output="screen" args="$(find console_path_parser)/config/my_arg_file.yaml"/>-->

 <node pkg="console_path_parser" type="console_path_parser_node" name="console_path_parser" respawn="false" output="screen">
        <rosparam command="load" file="$(find console_path_parser)/config/config_console_path_parser.yaml"/>
        <param name="path_folder" value="$(env ROS_BAG_FOLDER)/paths_from_console"/>
        <remap from="ID" to="ID_Comms"/>
</node>

</launch>
```

## Configurations
```
node_frequency: 5
path_folder: "../paths_from_console"
vehicle_id: 1
topics:
  subscribers:
    Mission_String: /Mission_String
    state: /nav_filter/state
    flag: /Flag
    gamma: /Gamma
    ID: /ID
  publishers:
    Path_Section: /Path_Section
    Formation: /Formation
    biased_formation: /biased_formation
    WPRef: /WPRef
    DepthRef: /ref/depth
    AltRef: /ref/altitude
    FullMission: /FullMission
```

