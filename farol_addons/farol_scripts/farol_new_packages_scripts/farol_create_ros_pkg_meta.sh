#!/bin/bash

#
# Developers: malaclemys          > jquintas@gmail.com
#
# Description: bash script for creating a ros meta package according with the farol stack best practices
#
# Note: This was greatly based on Oscar script. Thank you mexican god.
#
#@~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@
#@         ?                                      @
#@                  ___                           @
#@             ,,  // \\                          @
#@            (_,\/ \_/ \                         @
#@              \ \_/_\_/>                        @
#@              /_/  /_/              _______     @
#@                                   |START |     @
#@                                   |______|     @
#@                                   |            @
#@___________________________________|____________@
#

################################################################
# @.@ check that the number of received arguments are correct  #
################################################################
if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash farol_create_ros_pkg_meta.sh fiic_awesome_meta"
    exit
fi

################################################################
# @.@ assign first received argument to variable PKG_NAME      # 
################################################################
PKG_NAME_IN=${1,,}
PKG_NAME=$(echo "$PKG_NAME_IN" | sed -r 's/_([a-z])/\U\1/g')

################################################################
# @.@ Give some author love                                    # 
################################################################
AUTHOR_NAME="DSOR GROUP"
AUTHOR_EMAIL="dsor@gmail.com"
MAINTAINER_NAME="genX"
MAINTAINER_EMAIL="dsor@gmail.com"

################################################################
# @.@ setup file content                                       # 
################################################################
CMAKELISTS_CONTENT="cmake_minimum_required(VERSION 2.8.3)\nproject(${PKG_NAME_IN})\n\nfind_package(catkin REQUIRED)\n\ncatkin_metapackage()\n"
ROS_INDEPENDENT_CLASS_CONTENT="#!/usr/bin/env python\n\ndef my_generic_sum_function(a, b):\n    \"\"\"\n    TODO!! documentation\n    \"\"\"\n    return a + b"
PACKAGE_XML_CONTENT="<?xml version=\"1.0\"?>\n<package format=\"2\">\n  <name>${PKG_NAME_IN}</name>\n  <version>1.0.0</version>\n  <description>\n    TODO!\n  </description>\n\n  <license>GPLv3</license>\n\n  <author email=\"${AUTHOR_EMAIL}\">${AUTHOR_NAME}</author>\n  <maintainer email=\"${MAINTAINER_EMAIL}\">${MAINTAINER_NAME}</maintainer>\n\n  <buildtool_depend>catkin</buildtool_depend>\n\n</package>"
README_CONTENT="${PKG_NAME_IN} documentation"

###############################################################
# @.@ create folder structure                                 #  
###############################################################
mkdir -p ${PKG_NAME_IN}/${PKG_NAME_IN} 
cd ${PKG_NAME_IN}/${PKG_NAME_IN}

mkdir -p doc

################################################################
# @.@ create file structure                                    # 
################################################################
touch CMakeLists.txt
touch package.xml
touch doc/README.md

################################################################
# @.@ fill files with information from parameter section       #
################################################################
echo -e $CMAKELISTS_CONTENT  > CMakeLists.txt
echo -e $PACKAGE_XML_CONTENT > package.xml
echo -e $README_CONTENT > doc/README.md

################################################################
# @.@ Make or build the workspace, you decide                  #
################################################################
#catkin build --this
farol_cbt
cd ..

unset PKG_NAME
unset AUTHOR_NAME
unset AUTHOR_EMAIL
unset MAINTAINER_NAME
unset MAINTAINER_EMAIL
unset CMAKELISTS_CONTENT
unset ROS_INDEPENDENT_CLASS_CONTENT
unset PACKAGE_XML_CONTENT
unset README_CONTENT

################################################################
# @.@ Source bashrc and echo an ecouriging goodbye message     #
################################################################
source ~/.bashrc
echo "${PKG_NAME_IN} ros meta package created, you are now inside it. Let the fun begin..." 
unset PKG_NAME_IN
