# farol_bringup package

The `farol_bringup` is a ROS package that serves as the launching point of the entire farol stack. However, it does not act directly as the launcher, but instead it is used to replicate a bringup of our own to properly launch all the available vehicles. This can be attained using the following command:

```bash
dsor_pkg_bringup <PKG_NAME> <VEHICLE_NAME>
```

where `PKG_NAME` is the desired name for the bringup (to be added to `_bringup`), and `VEHICLE_NAME` is the desired vehicle to add to your bringup.You can add more vehicles later with the command:

```bash
dsor_add_vehicle_bringup <VEHICLE_NAME>
```

Inside the newly created bringup, you will find two main folders, the `config/` and the `launch/` folder. Each of them are further explained on their respective documentation tabs. However, it is better to tackle first the difference between the original `farol_bringup` and the replicated bringup.

## Config

In the `config/` folder reside the configuration files regarding the system in terms of what is general to all vehicles available in the bringup, on the `personal_ros.yaml` file in the `dev_config/` folder, and the configurations specific to each type of vehicle, inside the `vehicles/` folder.

### `personal.yaml` file

In here, you can edit the general parameters, like new topics and ports used for CPF communications, which are common to all types of vehicles. The vehicle within the topic should be replaced by the one instantiated when launching the system. The topics should resemble something like the following:

```yaml
addons/console_server:
    PORT: 7080

cooperative/cpf_wifi_server:
    broadcast_port: 2808

cooperative/cpf_wifi_client:
    broadcast_port: 2808

example_node:
    topics:
        publisher:
            example_pub: '/#vehicle#/example/example_pub'
        subscriber:
            example_sub: '/#vehicle#/example/example_sub'
```

This file is used to override the permanent version with default values inside the `farol_bringup` package. What really happens is that the permanent file is loaded and then the `personal_ros.yaml` file is also loaded. This means that if the same fields exist in both files, the fields specified in the `personal_ros.yaml` are the ones which are effectively loaded.

### Vehicle configuration files

Inside the `vehicles/` folder within your own bringup, reside the folders with respect to each type of vehicle available in the bringup. To override certain default parameters defined in the config files inside the `farol_bringup` stack, you need to create the right config files and write the new values you want to assign to the existing variables. For example, to assign new gains to the controllers, you just need to create the `control.yaml` file inside the folder of the vehicle you want to override. The following example demonstrates the overriding of the depth controller's PID gains:

```yaml
inner_loops_pid:
    controllers:
        depth:
            kp: 1.0
            ki: 1.0
            kd: 1.0
```

### `process.yaml` file

Finally, the process.yaml file needs to be configured so that the new config file inside the custom bringup is loaded. There are many processes defined within the file, although for now you just need to pay attention to the first one, named 'load':

```yaml
processes:
    - name: 'load' # Set to true to load all parameters need to run the stack
      launch_on_startup: true
      delay_before_start: 0.0
      cmd: 'roslaunch farol_bringup load_parameters.launch'
      args: # set to true when you want to add or override any default parameter(s)
            - common:= false
            - simulation:= false
            - addons:= false
            - navigation:= false
            - controllers:= false
            - cooperative:= false
            - comms:= false
            - acoustic_data:= false
            - planning:= false 
      dependencies:
            []
```

## Nodes

* [farol_bringup](./farol_bringup.md)

## Dependencies

* None
