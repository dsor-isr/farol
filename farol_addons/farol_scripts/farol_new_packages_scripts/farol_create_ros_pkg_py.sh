#!/bin/bash

# Description: bash script for creating a ros PY package according with the farol stack best practices

# @.@ assign first received argument to variable PKG_NAME 
if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash farol_create_ros_pkg_py.sh fiic_awesome_py"
fi

# @.@ assign first received argument to variable PKG_NAME
PKG_NAME_IN=${1,,}
PKG_NAME=$(echo "$PKG_NAME_IN" | sed -r 's/_([a-z])/\U\1/g')

# @.@ Give some author love
AUTHOR_NAME="#DSORTeam"
AUTHOR_EMAIL="dsor.isr@gmail.com"
MAINTAINER_NAME="#DSORTeam"
MAINTAINER_EMAIL="dsor.isr@gmail.com"


# @.@ setup file content 
CMAKELISTS_CONTENT="cmake_minimum_required(VERSION 3.1.0)
project(${PKG_NAME_IN})

find_package(catkin 
  # ROS components
  REQUIRED COMPONENTS
  std_msgs
  farol_msgs
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)
"

ROS_INDEPENDENT_CLASS_CONTENT="#!/usr/bin/env python

def my_generic_sum_function(a, b):
 \"\"\"
 THIS IS A DUMMY EXAMPLE
 \"\"\"
 return a + b
 "

PACKAGE_XML_CONTENT="<?xml version=\"1.0\"?>
<package format=\"2\">
 <name>${PKG_NAME_IN}</name>
 <version>1.0.0</version>
 <description>
 ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 </description>

 <license>MIT</license>

 <author email=\"dsor.isr@gmail.com\">#DSORTeam</author>
 <maintainer email=\"dsor.isr@gmail.com\">#DSORTeam</maintainer>

 <buildtool_depend>catkin</buildtool_depend>

 <!-- Add here ROS dependencies -->
 <depend>std_msgs</depend> 
 <depend>rospy</depend>
 <depend>farol_msgs</depend>

</package>
"

CONFIG_YAML_CONTENT="node_frequency: 10
"

README_CONTENT="${PKG_NAME_IN} documentation
"

LAUNCH_FILE_CONTENT="<?xml version=\"1.0\"?>
<launch>

 <node pkg=\"${PKG_NAME_IN}\" type=\"${PKG_NAME_IN}_node\" name=\"${PKG_NAME_IN}Node\" respawn=\"false\" output=\"screen\">
	<rosparam command=\"load\" file=\"\$(find ${PKG_NAME_IN})/config/config_${PKG_NAME_IN}.yaml\"/>
</node>

</launch>
"

SCRIPTS_CONTENT="#!/usr/bin/env python

import ${PKG_NAME_IN}_ros.${PKG_NAME^}Node

if __name__ == '__main__':
 ${PKG_NAME_IN}_ros.${PKG_NAME^}Node.main()
 "

MY_TEST_CONTENT="#TODO
"

SETUP_PY_CONTENT="#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['${PKG_NAME_IN}_algorithms', '${PKG_NAME_IN}_ros'],
 package_dir={'${PKG_NAME_IN}_algorithms': 'src/${PKG_NAME_IN}_algorithms', '${PKG_NAME_IN}_ros': 'src/${PKG_NAME_IN}_ros'}
)

setup(**d)
" 

ROS_NODE_CONTENT="#!/usr/bin/env python

\"\"\" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
\"\"\"
import rospy
from ${PKG_NAME_IN}_algorithms.${PKG_NAME^}Algorithm import my_generic_sum_function
from std_msgs.msg import Int8, Bool

class ${PKG_NAME^}Node():
    def __init__(self):
        \"\"\"
        Constructor for ros node
        \"\"\"

        \"\"\"
        @.@ Init node
        \"\"\"
        rospy.init_node('${PKG_NAME_IN}_node')

        
        \"\"\"
        @.@ Handy Variables
        # Declare here some variables you might think usefull -> example: self.fiic = true
        \"\"\"

        

        \"\"\"
        @.@ Dirty work of declaring subscribers, publishers and load parameters 
        \"\"\"
        self.loadParams()
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()


    \"\"\"
    @.@ Member Helper function to set up parameters; 
    \"\"\"
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)
    

    \"\"\"
    @.@ Member Helper function to set up subscribers; 
    \"\"\"
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for ${PKG_NAME^}Node')

    
    \"\"\"
    @.@ Member Helper function to set up publishers; 
    \"\"\"
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for ${PKG_NAME^}Node')


    \"\"\"
    @.@ Member helper function to set up the timer
    \"\"\"
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


    \"\"\"
    @.@ Member helper function to shutdown timer;
    \"\"\"
    def shutdownTimer(self):
        self.timer.shutdown()


    \"\"\"
    @.@ Timer iter callback. Where the magic should happen
    \"\"\"
    def timerIterCallback(self, event=None):
        # REMOVE pass and do your magic
        pass
            


def main():

    ${PKG_NAME_IN} = ${PKG_NAME^}Node()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
"

# @.@ create folder structure                                   
mkdir ${PKG_NAME_IN} 
cd ${PKG_NAME_IN}
mkdir -p config
mkdir -p docs
mkdir -p launch
mkdir -p scripts
mkdir -p src/${PKG_NAME_IN}_algorithms
mkdir -p src/${PKG_NAME_IN}_ros
mkdir -p test

# @.@ create file structure 
touch CMakeLists.txt
touch src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.py
touch package.xml
touch config/config_${PKG_NAME_IN}.yaml
touch docs/README.md
touch launch/${PKG_NAME_IN}.launch
touch scripts/${PKG_NAME_IN}_node
touch src/${PKG_NAME_IN}_ros/__init__.py
touch src/${PKG_NAME_IN}_algorithms/__init__.py
touch src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.py
touch test/${PKG_NAME_IN}_test.py
touch setup.py

# @.@ make python node executable:                              
chmod +x scripts/${PKG_NAME_IN}_node

# @.@ fill files with information from parameter section       
echo -e $CMAKELISTS_CONTENT$ > CMakeLists.txt
sed -i '$d' CMakeLists.txt 
echo -e $ROS_INDEPENDENT_CLASS_CONTENT$ > src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.py
sed -i '$d' src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.py 
echo -e $PACKAGE_XML_CONTENT$ > package.xml
sed -i '$d' package.xml 
echo -e $CONFIG_YAML_CONTENT$ > config/config_${PKG_NAME_IN}.yaml
sed -i '$d' config/config_${PKG_NAME_IN}.yaml 
echo -e $README_CONTENT$ > docs/README.md
sed -i '$d' docs/README.md 
echo -e $LAUNCH_FILE_CONTENT$ > launch/${PKG_NAME_IN}.launch 
sed -i '$d' launch/${PKG_NAME_IN}.launch 
echo -e $SCRIPTS_CONTENT$ > scripts/${PKG_NAME_IN}_node
sed -i '$d' scripts/${PKG_NAME_IN}_node 
echo -e $MY_TEST_CONTENT$ > test/${PKG_NAME_IN}_test.py
sed -i '$d' test/${PKG_NAME_IN}_test.py 
echo -e $SETUP_PY_CONTENT$ > setup.py
sed -i '$d' setup.py 
echo -e $ROS_NODE_CONTENT$ > src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.py
sed -i '$d' src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.py 

# @.@ Make or build the workspace, you decide                  
farol_cb

# @.@ Unset the created local variables of this file           
unset PKG_NAME
unset AUTHOR_NAME
unset AUTHOR_EMAIL
unset MAINTAINER_NAME
unset MAINTAINER_EMAIL
unset CMAKELISTS_CONTENT
unset PACKAGE_XML_CONTENT
unset CONFIG_YAML_CONTENT
unset README_CONTENT
unset LAUNCH_FILE_CONTENT
unset SCRIPTS_CONTENT
unset MY_TEST_CONTENT
unset ROS_NODE_CONTENT
unset ROS_INDEPENDENT_CLASS_CONTENT
unset SETUP_PY_CONTENT

# @.@ Source bashrc and echo an ecouriging goodbye message     
source ~/.bashrc
echo "${PKG_NAME_IN} ros py package created, you are now inside it. Let the fun begin..." 
unset PKG_NAME_IN