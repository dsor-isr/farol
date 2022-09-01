#!/bin/bash

NAME_OF_PKG_TO_CLEAN=${1}
MY_ROS_WORKSPACE=$ROS_WORKSPACE

if [[ -z "${NAME_OF_PKG_TO_CLEAN}// }" || "$#" -ne 1 ]]; then
    echo "Usage: clean_pkg_from_ws PACKAGE_NAME"
    return
elif [[ ${NAME_OF_PKG_TO_CLEAN} == *[/.]* ]]; then
    echo "Not supported using '/' or '.' in PACKAGE_NAME"
    return
fi

# remove log files -> not used
#rm -rf $MY_ROS_WORKSPACE/logs

# remove all build related files
rm -rf $MY_ROS_WORKSPACE/build/$NAME_OF_PKG_TO_CLEAN

# remove devel related files
rm -rf $MY_ROS_WORKSPACE/devel/share/$NAME_OF_PKG_TO_CLEAN
rm -rf $MY_ROS_WORKSPACE/devel/.private/$NAME_OF_PKG_TO_CLEAN
rm -rf $MY_ROS_WORKSPACE/devel/lib/pkgconfig/${NAME_OF_PKG_TO_CLEAN}.*
rm -rf $MY_ROS_WORKSPACE/devel/lib/lib${NAME_OF_PKG_TO_CLEAN}.so
rm -rf $MY_ROS_WORKSPACE/devel/lib/python2.7/dist-packages/${NAME_OF_PKG_TO_CLEAN}

# remove .catkin tools related files
rm -rf $MY_ROS_WORKSPACE/.catkin_tools/profiles/default/packages/$NAME_OF_PKG_TO_CLEAN

echo "any (build) trace of your pkg with name : $NAME_OF_PKG_TO_CLEAN was removed from your catkin_workspace"
