# farol_vx config - console_path_parser

## Run
This package is launched with farol stack
```
ex:
roslaunch farol_bringup farol_bringup.launch name:= mvector docks:=missions
```

Please check if **console_path_parser** is set to true in the vehicle configurations. For this, please check the following block in *farol_bringup/config/docks/mvector/process.yaml* file:

```yaml
 - name: 'addons'
        launch_on_startup: true
        delay_before_start: 0.0
        cmd: 'roslaunch farol_bringup addons.launch'
        args:
              - aggregator:= true
              - **console_path_parser:= true**
              - console_server:= true
              - wifi_acoustic_safety:= true
              - rosbridge_server:= false
```


## Launch file
You have a main launch (**farol_bringup.launch**) that calls a node (**farol_bringup_node**) responsible to manage all the processes in a **proecss.yaml** file.
The node while running is responsible to launch a subset of other launch files located at */farol_bringup/launch/dev_launch*. Among the exisitng ones, here the focus is in the **addons.launch** where **console_path_parser** is present.

Looking in some detail to the file we point out the key points:
```xml
    <arg name="name"                />   
    <arg name="mission"             default="example"   />   
    <arg name="wifi_acoustic_safety"     default="false"     />   <!-- safety features -->
    **<arg name="console_path_parser"         default="false"     />**   
    <arg name="http_server"         default="false"     />   
    <arg name="aggregator"          default="false"     />   
    <arg name="rosbridge_server"    default="false"     />   
    <arg name="websocket_port"      default="9090"      />       <!-- arg to use in http-->
```

By default all the nodes in the **farol_addons** package are set to false and won't run if you don't change the corresponding boolean in the **process.yaml** like in the previous section. 

```
 <group ns="addons">
```
Every addon is part of the group **addons**, which means that topics will be someting like */addons/node/some_topic*, here */addons/console_path_parser/some_topic*.

```xml
<!-- ############################ -->
<!-- Console Path Parser -->
<!-- ############################ -->
<group if="$(arg console_path_parser)">
    <node pkg="console_path_parser" type="console_path_parser_node" name="console_path_parser" respawn="false" output="screen">
        <param name="path_folder" value="$(env ROS_BAG_FOLDER)/paths_from_console"/>
    </node>
</group
```

As you can see the *if* guarantees that the node only starts if in the **process.yaml**, **console_path_parsert** is set to true.  


## Configurations

Configuration is divided between two files. The first one, *farol_bringup/config/dev_configs/ros.yaml*, is responsible with ros node items (frequency and topics names). Below, the section of **console_path_parser**:


```yaml
addons/console_path_parser:
    node_frequency: 5
    topics:
        subscribers: 
            Mission_String: /addons/Mission_String
            state: /nav/filter/state
            flag: /Flag
            gamma: /controls/gamma
            #ID: /ID
        publishers:
            Path_Section: /addons/path_section
            Formation: /addons/formation
            biased_formation: /addons/biased_formation
            WPRef: /addons/WPRef
            DepthRef: /ref/depth
            AltRef: /ref/altitude
            FullMission: /addons/full_mission
```
The second file, */farol_bringup/config/mission/vehicle/addons.yaml*, accommodates items more related with the node inner-workings. 
Below, the section of **console_path_parser**:
```yaml
*file, ex: /farol_bringup/config/docks/mvector/addons.yaml

console_path_parser:
    path_folder: "../paths_from_console" 
    vehicle_id: 3

```  

