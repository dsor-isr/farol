```
Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com
         poseiden    -> marcelo.jacinto@tecnico.ulisboa.pt

Descripton: Bash scripts for installing ROS farol stack and configuring OS for DSOR vehicles
Date: 20200226
Last update: 20200813

@~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@
@         ?                                               @
@          __                                             @
@      ____||_                  ___                       @
@     (_______)            ,,  // \\                      @  
@      ____||_            (_,\/ \_/ \                     @
@     (_______)             \ \_/_\_/>                    @
@                            /_/  /_/         ______      @
@                                            |START |     @
@                                            |______|     @
@                                            |            @
@____________________________________________|____________@
```

## Scripts

| File | Purpose| Usage | 
|:----------:|:----:|:---:| 
| **config_farol_vx.bash_vx** |  calls the other two scripts | bash config_farol.bash |
| **config_farol_vx_ros.bash** |  this script will deal with every ros configuration  | by using the previous script or directly calling: bash config_farol_ros.bash |
| **config_medus_vx_os.bash** |  Will configure a DSOR vehicle. That tiny details that are the glue for everything  | by using the first script or directly calling: bash config_farol_os.bash |

NOTE: no alias for this scripts so you should run them in their location. This will also be given by a shared link. Actually you probably will use this by downloading this folder from the shared link. The files are in the repo just for maintence purposes.

## config_farol_vx.bash details

Gives you the option to use or not the following scripts:
- config_farol_vx_ros.bash
- config_farol_vx_os.bash

## config_farol_vx_ros.bash details

When running it will:

 - Checks for internet connectivity to update and upgrade the operating system  
 - Requests your bitbucket credentials to clone the farol stack repository  
    - Asks if you want to ignore the farol_drivers package. Press y if will use the farol stack on simulation only  
    - Asks if you want to clone the console. It is used to plan missions, provide waypoints and visualize a 2D path, everything directly on a web browser  
    - Asks if you want to clone the documentation  
 - Verifies your Ubuntu distro and for installing the correct ROS versions
     - Noetic if Ubuntu 20.04 or Melodic if Ubuntu 18.04
     - Deals with all the installation procedures of ROS if you haven't ROS installed yet  
 - Installs external packages such as 
     - dmac, flexbe_behavior_engine, executive_smach, pip and python packages (mainly numpy, pandas, scipy, matplotlib, rospkg,roslib)  
 - Configures catkin_ws root workspace  
     - Initialize ROS workspace - initializes ROS  
 - Installs libraries such as 
     - libusb-dev, libudev-dev, libgeographic-dev, libxmlrcpp-dev, librosconsole-dev  
     - Installs extra ROS packages such as nmea_msgs, auv_msgs, rosparam_handler, geographic_msgs, rosbridge, dynamic reconfigure   
 - Add farol permanent alias to your bashrc  
 - Asks to install optional tools such as
     - exfat-fuse, terminator, tree, python-pygments, wmctrl, ntp, ntpdate, minicom, byobu, zip, screen
 - Add permissions to serial ports - important for testing sensors  
 - Compiles
     - ROS packages  
     - The farol stack code  

## config_farol_vx_os.bash details

When running it will:

- Enable auto-login and disable guest
    - guest user is unecessary, and we want to immediately start the system
- Remove shutdown from sudo  
    - important if we want to do poweroff from the mobile app
- Define settings of the wallpaper
    - maybe it is irrelevant, but ok
- Define settings of the Desktop Sharing
    - nothing to say here
- Change side of close button 
    - maybe it is irrelevant, but ok
- Disable screensaver
    - because yes
- Disable Power Management
    - don't need this, guessing batteries
- Grub timeout configurations 
    - We just want grub for 1 second
- Changing networks names to classical ones
    - new linux distros give a strange name to the network ports, this will switch to the classical ones (eth0, eth1 ...)
- Update Grub
    - all the previous changes only take effect after updating grub
- Add noatime to fstab 
    - just to give more life to the ssd disk
- Disable sleep timeout for static network
    - avoids hanging 2 minutes if some crazy happens with the network board
- Change Message of the day
    - just a detail when you ssh to the machine
- Change hosts file
    - tells the vehicle the existing ips in our network
- Create VehicleStartUp
    - adds the script that will start ROS as soon the vehicle boots
- Add udev rules
    - rename sensor ports basically
- Disable Network Manager
    - To avoid problems in the future
- Add  Network Configurations
    - Configurations of the ips of the vehicle
- Add bullet switch to system startup
    - allows to switch between internal and externall bullet
- Configure ntp client in vehicle
    - via ntp script

## general_install.bash details

When running it will:

- Clone the farol code, console and documentation repository
    - get all the necessary code
- Install basic programs used by every developer
    - vim, terminator, etc
- Clone external repositories not developed in DSOR
    - code such as auv_msgs, etc.
- Install ROS 1
    - install ros melodic or noetic depending on the ubuntu version the script is run on
- Python 2 and 3 setup
    - setup python2 and python3 along with pip2 and pip3 to run both older packages and newer ones
- Run costume scripts 
    - install the script /farol_scripts/system_configurations/hosts_files/hosts_cpy.sh
- Install rospackages needed
    - installed using sudo apt-get install ros-version-package
- Install external libraries that need to be installed manually
    - installed geographic-lib
- Add alias to the development environement (such as farol_cm)
    - add the alias in /farol_scripts/system_configurations/easy_alias/farol_permanent_alias/alias.sh to .bashrc file
