# farol_bringup package

farol_bringup is a ROS package that serves as the launching point of the entire farol stack. However, it does not act directly as the launcher, but instead it is used to replicate a bringup of our own to properly launch all the available vehicles.

![farol_bringup Diagram](img/farol_bringup_diagram.png)

## Rationale

This package serves as the launcher for the entire stack, as it was mentioned before, being replicated 
where configuration setups are defined, such as default value overriding

## Package Content

![http_server struct](img/console_server_structure.png)

## Code documentation

[source](http://lungfish.isr.tecnico.ulisboa.pt/farol_vx_doxy/farol_addons/http_server/html/index.html)

## Using http_server

[Examples](./pages.html)

## Requirements

* http server
* web console (Yebisu)
