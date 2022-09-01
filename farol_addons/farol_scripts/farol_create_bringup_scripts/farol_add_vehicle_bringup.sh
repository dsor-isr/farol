#!/bin/bash

# Description: bash script for setup a new vehicle in your bringup package

# @.@ check if two arguments are received (name of the vehicle that you will use)
# Note: check farol repository inside DSOR organization to see which vehicle are available

if [ "$#" != "1" ]; then
  echo "Illegal number of parameters. You must pass 1 argument (vehicle name)"
  return 0
fi

# @.@ assign second received arguments to variable VEHICLE_NAME
VEHICLE_NAME=${1,,} # convert the argument to lowercase

# @.@ Give some author love
AUTHOR_NAME="DSOR GROUP"
AUTHOR_EMAIL="dsor.isr@gmail.com"
MAINTAINER_NAME="DSORTeam"
MAINTAINER_EMAIL="dsor.isr@gmail.com"

# @.@ create folder structure
mkdir ${VEHICLE_NAME}

# @.@ create file structure
cp ../../../farol/farol_bringup/config/defaults/$VEHICLE_NAME/process.yaml $VEHICLE_NAME/

# @.@ Unset the created local variables of this file
unset VEHICLE_NAME
unset AUTHOR_NAME
unset AUTHOR_EMAIL
unset MAINTAINER_NAME
unset MAINTAINER_EMAIL

# @.@ Source bashrc
source ~/.bashrc