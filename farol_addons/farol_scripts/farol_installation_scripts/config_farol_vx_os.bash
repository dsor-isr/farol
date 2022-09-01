#!/bin/bash

#
# Developer: malaclemys -> jquintas@gmail.com
#            DSOR Team
# 
# Description: Configure OS farol system for DSOR vehicles. 
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
echo '##########################################################################'
echo '    __  ___         __                      ____  _____     __            '                    
echo '   /  |/  /__  ____/ /_  ___________ _     / __ \/ ___/    _||____        '     
echo '  / /|_/ / _ \/ __  / / / / ___/ __ `/    / / / /\__ \    (_______)       '     
echo ' / /  / /  __/ /_/ / /_/ (__  ) /_/ /    / /_/ /___/ /     _||____        '     
echo '/_/  /_/\___/\__,_/\__,_/____/\__,_/     \____//____/     (_______)       '      
echo '                                                                          '
echo '######################### To the moon and back ###########################'
echo ''

# Get the location of the farol_real_scripts package
export FAROL_REAL_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname farol_real_scripts | head -n 1)
export FAROL_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname farol_scripts | head -n 1)

###################################################
# @.@ Enable auto-login and disable guest         #
###################################################

echo ''
echo '################################################'
echo '# Enabling auto-login and disabling guest user #'
echo '################################################'
echo ''
#sudo systemctl edit getty@tty1.service
sudo mkdir /etc/systemd/system/getty@tty1.service.d
sudo touch /etc/systemd/system/getty@tty1.service.d/override.conf
sudo bash -c "echo '[Service]' >> /etc/systemd/system/getty@tty1.service.d/override.conf"
sudo bash -c "echo 'ExecStart=' >> /etc/systemd/system/getty@tty1.service.d/override.conf"
sudo bash -c "echo 'ExecStart=-/sbin/agetty --noissue --autologin ${USER} %I \$TERM' >> /etc/systemd/system/getty@tty1.service.d/override.conf"
sudo bash -c "echo 'Type=idle' >> /etc/systemd/system/getty@tty1.service.d/override.conf"
sleep 2

###################################################
# @.@ Remove shutdown from sudo                   #
###################################################

echo ''
echo '################################################'
echo '# Removing shutdown from sudo                  #'
echo '################################################'
echo ''

sudo chmod a+s /sbin/shutdown || true
sleep 2

###################################################
# @.@  Grub timeout configurations                #
###################################################

echo ''
echo '################################################'
echo '# Configuring grub timeout                     #'
echo '################################################'
echo ''

# *.* Comment this line if it is not already commented
sudo sed -i -e 's/GRUB_HIDDEN_TIMEOUT=0/#GRUB_HIDDEN_TIMEOUT=0/g' /etc/default/grub
# *.* if hidden timeout is present, comment it because it is deprecated
sudo sed -i -e 's/GRUB_HIDDEN_TIMEOUT_QUIET/#GRUB_HIDDEN_TIMEOUT_QUIET/g' /etc/default/grub
# *.* Change timeout to just 1 second
sudo sed -i -e 's/GRUB_TIMEOUT=10/GRUB_TIMEOUT=1/g' /etc/default/grub

sleep 2

###################################################
# @.@  Changing networks names to classical ones  #
###################################################

echo ''
echo '################################################'
echo '# Changing networks names to classical ones    #'
echo '################################################'
echo ''

# *.* This will change the network name to the classical eth0 ...
sudo sed -i -e 's/GRUB_CMDLINE_LINUX=""/GRUB_CMDLINE_LINUX="net.ifnames=0 biosdevname=0"/g' /etc/default/grub
sleep 2

###################################################
# @.@  Update Grub                                #
###################################################

echo ''
echo '################################################'
echo '# Updating Grub                                #'
echo '################################################'
echo ''

sudo update-grub
sleep 2

###################################################
# @.@  Add noatime to fstab                       #
###################################################

echo ''
echo '############################################################'
echo '# Adding noatime to fstab for spanning the life of the ssd #'
echo '############################################################'
echo ''

sudo sed -i -e 's/errors/noatime,errors/g' /etc/fstab
sleep 2

###################################################
# @.@  Disable sleep timeout for static network   #
###################################################

echo ''
echo '######################################################'
echo '# Disabling sleep timeout in /etc/init/failsave.conf #'
echo '######################################################'
echo ''

sudo sed -i -e 's/sleep/#sleep/g' /etc/init/failsafe.conf
sleep 2

###################################################
# @.@  Change Message of the day                  #
###################################################

echo ''
echo '######################################################'
echo '# Changing message of the day motd                   #'
echo '######################################################'
echo ''

sudo rm -f /etc/motd
sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/motd_banners/motd_farol_genX" /etc/motd
sleep 2


###################################################
# @.@  Create VehicleStartUp                      #
###################################################

echo ''
echo '######################################################'
echo '# Creating vehicle (farol/delfim/DS/fusion) startup #'
echo '######################################################'
echo ''

case $HOSTNAME in
    mred | myellow | mblack | muned | mvector)
    echo "You are configuring the $HOSTNAME vehicle. FarolStartUp will be added to the system"
    sudo rm -f /etc/init.d/FarolStartUp
    sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/services_init_scripts/FarolStartUp" /etc/init.d/FarolStartUp
    chmod +x /etc/init.d/FarolStartUp
    sudo update-rc.d FarolStartUp defaults 99
    sudo touch /etc/default/FarolStartUp
    echo 'CATKIN_ROOT'=${CATKIN_ROOT} | sudo tee -a /etc/default/FarolStartUp
    ;;

    delfim)
    echo "You are configuring the $HOSTNAME vehicle. DelfimStartUp will be added to the system"
    sudo rm -f /etc/init.d/DelfimStartUp
    sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/services_init_scripts/DelfimStartUp" /etc/init.d/DelfimStartUp
    chmod +x /etc/init.d/DelfimStartUp
    sudo update-rc.d DelfimStartUp defaults 99
    sudo touch /etc/default/DelfimStartUp
    echo 'CATKIN_ROOT'=${CATKIN_ROOT} | sudo tee -a /etc/default/DelfimStartUp

    ;;

    mds-lower)
    echo "You are configuring the $HOSTNAME vehicle. mds-lower_startup will be added to the system"
    sudo rm -f /etc/init.d/mds-lower_startup
    sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/services_init_scripts/mds-lower_startup" /etc/init.d/mds-lower_startup
    chmod +x /etc/init.d/mds-lower_startup
    sudo update-rc.d mds-lower_startup defaults 99
    sudo touch /etc/default/mds-lower_startup
    echo 'CATKIN_ROOT'=${CATKIN_ROOT} | sudo tee -a /etc/default/mds-lower_startup
    ;;

    mds-upper)
    echo "You are configuring the $HOSTNAME vehicle. mds-upper_startup will be added to the system"
    sudo rm -f /etc/init.d/mds-upper_startup
    sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/services_init_scripts/mds-upper_startup" /etc/init.d/mds-upper_startup
    chmod +x /etc/init.d/mds-upper_startup
    sudo update-rc.d mds-upper_startup defaults 99
    sudo touch /etc/default/mds-upper_startup
    echo 'CATKIN_ROOT'=${CATKIN_ROOT} | sudo tee -a /etc/default/mds-upper_startup
    ;;

    hrov-pts)
    echo "You are configuring the $HOSTNAME vehicle. HrovStartUp will be added to the system"
    sudo rm -f /etc/init.d/mds-upper_startup
    sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/services_init_scripts/HrovStartUp" /etc/init.d/HrovStartUp
    chmod +x /etc/init.d/HrovStartUp
    sudo update-rc.d HrovStartUp defaults 99
    sudo touch /etc/default/HrovStartUp
    echo 'CATKIN_ROOT'=${CATKIN_ROOT} | sudo tee -a /etc/default/HrovStartUp
    ;;

    fusion)
    echo "You are configuring the $HOSTNAME vehicle. FusionStartUp will be added to the system"
    sudo rm -f /etc/init.d/FusionStartUp
    sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/services_init_scripts/FusionStartUp" /etc/init.d/FusionStartUp
    chmod +x /etc/init.d/FusionStartUp
    sudo update-rc.d FusionStartUp defaults 99
    sudo touch /etc/default/FusionStartUp
    echo 'CATKIN_ROOT'=${CATKIN_ROOT} | sudo tee -a /etc/default/FusionStartUp
    ;;

    *)
    echo "$HOSTNAME is not a DSOR vehicle or there is not startup script. If the latter is true you can make one and add it to this script"
    ;;
esac

sleep 2

###################################################
# @.@  Add udev rules                             #
###################################################

echo ''
echo '######################################################'
echo '# Adding udev rules                                  #'
echo '######################################################'
echo ''

sudo rm -f /etc/udev/rules.d/50-udev.rules
sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/udev_rules/${HOSTNAME}_vx_50-udev.rules" /etc/udev/rules.d/50-udev.rules

sleep 2

###########################################
# @.@  Copy file hosts                    #
###########################################

echo ''
echo '######################################################'
echo '# Copy File Hosts                                    #'
echo '######################################################'
echo ''

${FAROL_REAL_SCRIPTS}/system_configurations/hosts_files/hosts_cpy.sh
sleep 2

###################################################
# @.@  Installing bullet switch script            #
###################################################

echo ''
echo '######################################################'
echo '# Adding bullet switch to system startup             #'
echo '######################################################'
echo ''

source "${FAROL_REAL_SCRIPTS}/system_configurations/bullet_switch/install_bullet.bash"

sleep 2

###################################################
# @.@  Add Network Configurations                #
###################################################

echo ''
echo '######################################################'
echo '# Adding  Network configuratiosn                     #'
echo '######################################################'
echo ''

sudo rm -f /etc/netplan/01-netcfg.yaml /etc/netplan/*.yaml
sudo ln -s "${FAROL_REAL_SCRIPTS}/system_configurations/network_interfaces/${HOSTNAME}_vx_netplan_config.yaml" /etc/netplan/01-netcfg.yaml
sudo netplan apply

###################################################
# @.@  Configure ntp client in vehicle            #
###################################################

echo ''
echo '######################################################'
echo '# Configuring ntp client in vehicle             #'
echo '######################################################'
echo ''

sudo bash ${FAROL_SCRIPTS}/ntp_scripts/config_ntp.sh client
sleep 2

###################################################
# @.@  Give dialout group permissions             #
###################################################

echo ''
echo '######################################################'
echo '# Configuring dialout permissions                    #'
echo '######################################################'
echo ''

sudo usermod -a -G dialout $USER

# Sometimes this fails, just double checking
sudo apt-get install libxmlrpcpp-dev
sleep 2

###################################################
# @.@  Bye                                       #
###################################################

echo ''
echo 'Thank you, from the DSOR Team! Have a hell of a ride at a water site near you.'
echo ''
echo 'System will reboot in 5s...'
echo ''
sleep 4
echo 'Bye!'
sleep 1
sudo reboot 0