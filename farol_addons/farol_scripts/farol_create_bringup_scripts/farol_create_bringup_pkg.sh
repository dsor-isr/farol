#!/bin/bash

# Description: bash script for creating a custom bringup package 

# @.@ check if two arguments are received (1 - name of the bringup package | 2 - name of the vehicle that you will use)
# Note: check farol repository inside DSOR organization to see which vehicle are available

if [ "$#" != "2" ]; then
  echo "Illegal number of parameters. You must pass 2 arguments (package name & vehicle name)"
  return 0
fi

# @.@ assign first received arguments to variable PKG_NAME
PKG_NAME=${1,,}"_bringup" # convert the argument to lowercase
# @.@ assign second received arguments to variable VEHICLE_NAME
VEHICLE_NAME=${2,,}

# @.@ Give some author love
AUTHOR_NAME="#DSORTeam"
AUTHOR_EMAIL="dsor.isr@gmail.com"
MAINTAINER_NAME="#DSORTeam"
MAINTAINER_EMAIL="dsor.isr@gmail.com"

# @.@ Setup file content
START_SCENARIO_CONTENT="<?xml version=\"1.0\"?>
<launch>
    <!-- Flags to select GUI, frame of reference, vehicle and world-->
    <arg name=\"gui\"            default=\"true\"/>
    <arg name=\"use_sim_time\"   default=\"false\"/>
    <arg name=\"world_frame\"    default=\"world\"/>

    <!-- Choose the world to launch (default is expo_lisbon)-->
    <arg name=\"folder\" default=\"expo_lisbon_worlds\" />
    <arg name=\"world\"  default=\"expo_lisbon\" />

    <!-- Select the gazebo world -->
    <include file=\"\$(find farol_worlds)/launch/\$(arg folder)/\$(arg world).launch\">
        <arg name=\"gui\" value=\"\$(arg gui)\"/>
        <arg name=\"use_sim_time\" value=\"\$(arg use_sim_time)\"/>
    </include>
</launch>
"

START_VEHICLE_CONTENT="<?xml version=\"1.0\"?>
<launch>
    <!-- Parameters and Arguments -->
    <arg name=\"name\"               default=\"${VEHICLE_NAME}\" />     <!-- Name of the vehicle being launched --> 
    <arg name=\"id\"                 default=\"0\" />                   <!-- Number of the vehicle -->
    <arg name=\"config_package\"     default=\"${PKG_NAME}\"/>  <!-- Name of the package where the configuration files are stored -->
    <arg name=\"folder\"             default=\"vehicles\" />            <!-- Name of the folder for the configuration files -->     

    <arg name=\"dollar\" value=\"$\" />
    <arg name=\"config_package_path\"  value=\"(find \$(arg config_package)\" />

    <!-- Launch the Processes in process_gazebo -->
    <group ns=\"\$(arg name)\$(arg id)\">

        <node pkg=\"farol_bringup\" type=\"farol_bringup_node\" name=\"farol_bringup\" respawn=\"false\" output=\"screen\">
            <rosparam command=\"load\" file=\"\$(arg dollar)\$(arg config_package_path))/config/\$(arg folder)/\$(arg name)/process.yaml\"/>
            <param name=\"name\"                  value=\"\$(arg name)\"/>
            <param name=\"vehicle_id\"            type=\"int\" value=\"\$(arg id)\" />
            <param name=\"config_package_path\"   value=\"\$(arg config_package_path)\"/>
            <param name=\"folder\"                value=\"\$(arg folder)\" />
            <param name=\"namespace\"             value=\"true\"/>
            <param name=\"process_state_publish_rate\" type=\"double\" value=\"0.33\" />
        </node>
    </group>
</launch>
"

README_CONTENT="${PKG_NAME} documentation"

MKDOCS_CONTENT=""

DEFAULT_CONSOLE_PORT=7080
DEFAULT_CPF_BROADCAST_PORT=2808

# Check if the console_port (environment variable) is defined, if not set with the default value
[ -z "$console_port" ] && CONSOLE_PORT=$DEFAULT_CONSOLE_PORT || CONSOLE_PORT=$console_port

# Check if the cpf_broadcast_port (environment variable) is defined, if not set with the default value
[ -z "$cpf_broadcast_port" ] && CPF_BROADCAST_PORT=$DEFAULT_CPF_BROADCAST_PORT || CPF_BROADCAST_PORT=$cpf_broadcast_port

PERSONAL_ROS_CONTENT="addons/console_server:
    PORT: $CONSOLE_PORT

cooperative/cpf_wifi_server:
    broadcast_port: $CPF_BROADCAST_PORT

cooperative/cpf_wifi_client:
    broadcast_port: $CPF_BROADCAST_PORT
"

CMAKELISTS_CONTENT="cmake_minimum_required(VERSION 3.1)
project(${PKG_NAME})

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

include_directories(\${catkin_INCLUDE_DIRS})
"

PACKAGE_XML_CONTENT="<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>${PKG_NAME}</name>
  <version>1.0.0</version>
  <description>The ${PKG_NAME} package</description>

  <author email=\"dsor.isr@gmail.com\">#DSORTeam</author>
  <maintainer email=\"dsor.isr@gmail.com\">#DSORTeam</maintainer>

  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

</package>
"

# @.@ create folder structure
mkdir ${PKG_NAME}
cd ${PKG_NAME}
mkdir -p config/dev_configs/.ros_tmp
mkdir -p config/vehicles/${VEHICLE_NAME}
mkdir -p docs
mkdir -p launch

# @.@ create file structure
touch config/dev_configs/personal_ros.yaml
touch config/dev_configs/.ros_tmp/.gitkeep
cp ../farol/farol_bringup/config/defaults/$VEHICLE_NAME/process.yaml config/vehicles/${VEHICLE_NAME}/
touch docs/README.md
touch launch/start_scenario.launch
touch launch/start_vehicle.launch
touch CMakeLists.txt
touch mkdocks.yml
touch package.xml

# @.@ fill files with information from parameter section
echo -e $START_SCENARIO_CONTENT$ > launch/start_scenario.launch
sed -i '$d' launch/start_scenario.launch
echo -e $START_VEHICLE_CONTENT$ > launch/start_vehicle.launch
sed -i '$d' launch/start_vehicle.launch 
echo -e $README_CONTENT$ > docs/README.md
sed -i '$d' docs/README.md 
echo -e $CMAKELISTS_CONTENT$ > CMakeLists.txt
sed -i '$d' CMakeLists.txt 
echo -e $MKDOCS_CONTENT$ > mkdocks.yml
sed -i '$d' mkdocks.yml 
echo -e $PACKAGE_XML_CONTENT$ > package.xml
sed -i '$d' package.xml 
echo -e $PERSONAL_ROS_CONTENT$ > config/dev_configs/personal_ros.yaml 
sed -i '$d' config/dev_configs/personal_ros.yaml

# @.@ Discard locally tracking in git the presonal_ros.yaml file
git update-index --assume-unchanged config/dev_configs/personal_ros.yaml

# @.@ Build the workspace
farol_cbt

# @.@ Unset the created local variables of this file
unset PKG_NAME
unset VEHICLE_NAME
unset AUTHOR_NAME
unset AUTHOR_EMAIL
unset MAINTAINER_NAME
unset MAINTAINER_EMAIL
unset START_SCENARIO_CONTENT
unset START_VEHICLE_CONTENT
unset README_CONTENT
unset CMAKELISTS_CONTENT
unset MKDOCS_CONTENT
unset PACKAGE_XML_CONTENT
unset PERSONAL_ROS_CONTENT


# @.@ Source bashrc
source ~/.bashrc