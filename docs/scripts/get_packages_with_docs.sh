#!/usr/bin/env bash

# Script that outputs to STDOUT the name of the packages with documentation (and that are not inside the ignore_packages.txt)

# Set the ROS workspace to a default value if not set
if [[ -z "${ROS_WORKSPACE}" ]]; then
  ROS_WORKSPACE=${HOME}/catkin_ws
fi

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source ${ROS_WORKSPACE}/devel/setup.bash

# Get the list of packages to skip during unit testing
ignore_packages_file='docs/ignore_packages.txt'
ignore_packages=()

# Read the packages to ignore to a list
while IFS= read -r line || [[ -n "$line" ]]; do
    # reading each line
    ignore_packages+=($line)
done < "$ignore_packages_file"

# Save the current directory
curr_dir=$(pwd)

# Get the catkin workspace absolute directory
workspace_abs_path=$(roscd && cd src && pwd)

# Iterate through list of ROS packages in the current workspace
for package in $(catkin list); do

    # Check if the package is not in the ignore list
    if ! [[ ${ignore_packages[*]} =~ (^|[[:space:]])"${package}"($|[[:space:]]) ]] && ! [[ '-' =~ (^|[[:space:]])"${package}"($|[[:space:]]) ]]; then

        # Go to the ROS package
        roscd ${package}

        # Get the package absolute path
        package_abs_path=$(pwd)

        # Get the package path relative to the workspace directory
        package_path=${package_abs_path//$workspace_abs_path}

        # Check if the package has a doc directory
        if [[ -d "docs" ]] && [[ -f "mkdocs.yml" ]]; then
            # Create a symbolic link for the documentation folders inside the 
            echo ${package_path}
        fi

    fi
done

# Go back to the original directory
cd ${curr_dir}