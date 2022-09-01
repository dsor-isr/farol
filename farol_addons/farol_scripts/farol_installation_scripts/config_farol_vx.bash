#!/bin/bash

#
# Developer: malaclemys -> jquintas@gmail.com
#            DSOR Team
# 
# Description: Configure ROS farol stack and OS dsor vehicle. 
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


echo ''
echo '################# Welcome to #######################'
echo '    __  ___         __                      __      '                    
echo '   /  |/  /__  ____/ /_  ___________ _     _||____  '     
echo '  / /|_/ / _ \/ __  / / / / ___/ __ `/    (_______) '     
echo ' / /  / /  __/ /_/ / /_/ (__  ) /_/ /      _||____  '     
echo '/_/  /_/\___/\__,_/\__,_/____/\__,_/      (_______) '     
echo '                                                    '
echo '############### Enjoy the ride #####################'
echo ''



###################################################
# @.@ Config ROS and OS for farol                #
###################################################

ros_configured=0

# *.* Config ROS
read -p " Do you wish to config ROS Farol stack? <y/N> " prompt_1
if [[ $prompt_1 =~ [yY](es)* ]]; then
    source 'config_farol_vx_ros.bash'
    ros_configured=1
fi

sleep 1

# *.* Config OS, only necessary for real vehicles and ROS must be installed
read -p " Do you wish to config OS Farol stack? <y/N> " prompt_2
if [[ $prompt_2 =~ [yY](es)* ]]; then
    while [[ $ros_configured != 1 ]]
    do 
        if grep -q 'CATKIN_ROOT' ~/.bashrc; then
            echo "Everything looks nice to go. Let's start the OS configuration"
            sleep 1
            source 'config_farol_vx_os.bash'
            ros_configured=1
        else
            echo ''
            echo "WARN: You need to have Farol Ros stack configured in your system"
            echo "Apparently there is no Farol ROS stack in your system."
            echo ''
            read -p "Please config ROS Farol stack? <y/N> " prompt_1
            if [[ $prompt_1 =~ [yY](es)* ]]; then
                source 'config_farol_vx_ros.bash'
                ros_configured=1
            else
                echo "Sorry you really need Farol ROS stack. Here we go again!"
                ros_configured=0
            fi
        fi
    done
fi

if [[ $prompt_1 =~ [yY](es)* ]] || [[ $prompt_2 =~ [yY](es)* ]]; then 
    echo ''
    echo 'Thanks from DSOR Team.'
    echo ''
fi
