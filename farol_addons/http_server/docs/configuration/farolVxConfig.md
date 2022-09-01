# farol_vx config - http_server 

## Run

### Vehicle Side
This package is launched with farol stack
```
ex:
roslaunch farol_bringup farol_bringup.launch name:= mvector docks:=missions
```

Please check if **console_server** is set to true in the vehicle configurations. For this, please check the following block in *farol_bringup/config/docks/mvector/process.yaml* file:

```yaml
 - name: 'addons'
        launch_on_startup: true
        delay_before_start: 0.0
        cmd: 'roslaunch farol_bringup addons.launch'
        args:
              - aggregator:= true
              - console_path_parser:= true
              - **console_server:= true**
              - wifi_acoustic_safety:= true
              - rosbridge_server:= false
```


## Launch file
You have a main launch (**farol_bringup.launch**) that calls a node (**farol_bringup_node**) responsible to manage all the processes in a **proecss.yaml** file.
The node while running is responsible to launch a subset of other launch files located at */farol_bringup/launch/dev_launch*. Among the exisitng ones, here the focus is in the **addons.launch** where **console_server** is present.

Looking in some detail to the file we point out the key points:
```xml
    <arg name="name"                />   
    <arg name="mission"             default="example"   />   
    **<arg name="wifi_acoustic_safety"     default="false"     />**   <!-- safety features -->
    <arg name="console_path_parser"         default="false"     />   
    <arg name="console_server"         default="false"     />   
    <arg name="aggregator"          default="false"     />   
    <arg name="rosbridge_server"    default="false"     />   
    <arg name="websocket_port"      default="9090"      />       <!-- arg to use in http-->
```

By default all the nodes in the **farol_addons** package are set to false and won't run if you don't change the corresponding boolean in the **process.yaml** like in the previous section. 

```
 <group ns="addons">
```
Every addon is part of the group **addons**, which means that almost all topics will be someting like */addons/node/some_topic*, here */addons/http_server/some_topic*.

```xml
<!-- ############################ -->
<!-- HTTP Server for Console -->
<!-- ############################ -->
<group if="$(arg console_server)">
    <node pkg="http_server" type="console.py" name="console_server" args="console" output="screen">
        <param name="Mission_Folder" value="$(find http_server)/../../Missions_FOLDER" />
        <param name="pages_folder"   value="$(find http_server)/pages/" />
    </node>
</group>
```

As you can see the *if* guarantees that the node only starts if in the **process.yaml**, **console_server** is set to true.  


## Configurations

Note that some configurations, in this case, are set in the launch file:

```xml
<param name="Mission_Folder" value="$(find http_server)/../../Missions_FOLDER" />
<param name="pages_folder"   value="$(find http_server)/pages/" />
```

Other configurations are at */farol_bringup/config/mission/vehicle/addons.yaml*:
```yaml
*file, ex: /farol_bringup/config/docks/mvector/addons.yaml
console_server:
    PORT: 7080
    ROOT_NAMESPACE: true
```

