#!/bin/bash

if [[ $# -ne 4 ]]; then
    echo "Illegal number of parameters"
    exit 2
fi

rosservice call /inner_forces/change_inner_gains "inner_type: '$1'
kp: $2 
ki: $3 
kd: $4"