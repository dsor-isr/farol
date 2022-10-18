############################################################
# -- farol alias: used to save time in typing commands -- #
############################################################

# source devel setup.bash
source ${ROS_WORKSPACE}/devel/setup.bash

# enable git completition
source ${FAROL_SCRIPTS}/farol_scripts_for_bash/git-completion.bash

# display git branch on console prompt
source ${FAROL_SCRIPTS}/farol_scripts_for_bash/display_git_branch_in_prompt.sh

# git pull repo and get the latest updates in the corresponding submodules
source ${FAROL_SCRIPTS}/farol_scripts_for_bash/git_special_commands.sh

# roscat cat a file by pkg_name and filename
source ${FAROL_SCRIPTS}/farol_scripts_for_bash/roscat.sh

# source autocomplete extensions
source ${FAROL_SCRIPTS}/farol_scripts_for_bash/autcomplete.sh

# source transfer.sh
source ${FAROL_SCRIPTS}/farol_scripts_for_bash/transfer.sh

#####################
# -- ROS related -- #
#####################

# kill all nodes
alias kill_all_ros_nodes='sudo pkill -f ros' 

# view state machine
alias smach_viewer='rosrun smach_viewer smach_viewer.py'

# open rviz visualization
alias rviz='rosrun rviz rviz'

# view current tf frames
alias tf_view_frames='cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &'

# clean package from workspace
alias clean_pkg_from_ws='source ${FAROL_SCRIPTS}/farol_scripts_for_bash/clean_pkg_from_ws.sh'

# clean logs from ros, handy when over 1gb message appears
alias clean_ros_logs='rosclean purge -y'


#####################
# -- compilation -- #
#####################

# build all the code
alias farol_cb='roscd; catkin build; cd $OLDPWD' # build the entire catkinworkspace (you need to be somewhere inside your catkin workspace)

# build only one package
alias farol_cbt='catkin build --this' # build one pkg (you need to be somewhere inside your ros pkg)

# compile all the farol stack
alias farol_cm='roscd; catkin_make; cd $OLDPWD'

alias farol_cb_vim='roscd; bash ${FAROL_SCRIPTS}/farol_scripts_for_bash/farol_build_vim.bash'

############################
# -- common handy tools -- #
############################

# when typing fast sometimes ls gets typed as sl
alias sl='ls'

# replace cat with python-pygments to cat with colors
alias cap='pygmentize -g'

# going back one directory and showing files convenient alias
alias ..='cd .. && ls'

# toggle terminal from restored to maximized
alias m='wmctrl -r :ACTIVE: -b toggle,maximized_vert,maximized_horz'

# shutdown pc
alias poweroff='sudo shutdown -h now'

# different way to shutdown pc
alias pcdown='sudo shutdown -P now'

# restart the pc
alias pcrestart='sudo shutdown -r now'

# That anoying moments you forgot sudo
alias please_fiic='sudo $(history -p !!)' # run last command as sudo

# source your bashrc
alias S='source ${HOME}/.bashrc'

# give some sanity to your shell
alias sane_your_shell='stty sane; clear;'

######################
# -- ntpdate sync -- #
######################

# farol ntpdate with console pc
alias farol_ntpdate='sudo ntpdate -bu ntpServer'

# ntpdate with the world
alias world_ntpdate='sudo service ntp stop && sudo ntpdate -s time.nist.gov && sudo service ntp start'

#####################
# -- Text editor -- #
#####################

# remove automatically spaces at the end of files, needs the file as argument at the end, i.e. remove_spaces my_file.txt
alias remove_endline_spaces="sed -i 's/\s*$//'"

# to clean temp files *.~ recursively
alias clean_temp_files='find . -name "*~" -type f -exec /bin/rm -fv -- {} +'

######################
# -- Change gains -- #
######################
alias farol_change_inners_gains='bash ${FAROL_SCRIPTS}/farol_scripts_for_bash/change_inner_forces_gains.bash'
alias farol_change_pfollowing_gains='bash ${FAROL_SCRIPTS}/farol_scripts_for_bash/change_pfollowing_gains.bash'

##########################
# -- ROS FAROL PATHS -- #
##########################
export ROS_BAG_FOLDER="${CATKIN_ROOT}"
export ROS_PACKAGE_PATH="${ROS_WORKSPACE}/src:/opt/ros/${ROS_DISTRO}/share"

if [ -d "$(find ${ROS_WORKSPACE}/src/ -type d -iname farol ! -path '*.git*')" ]; then
	export ROS_LOCATIONS="farol=$(find ${ROS_WORKSPACE}/src/ -type d -iname farol ! -path '*.git*')"
fi
if [ -d "${ROS_WORKSPACE}/src/farol_real" ]; then
	export ROS_LOCATIONS="farol_real=${ROS_WORKSPACE}/src/farol_real:${ROS_LOCATIONS}"
fi
if [ -d "${ROS_WORKSPACE}/src/farol_simulation" ]; then
	export ROS_LOCATIONS="farol_simulation=${ROS_WORKSPACE}/src/farol_simulation:${ROS_LOCATIONS}"
fi
export ROSCONSOLE_FORMAT='[${severity}] [${time}]: ${node}: ${message}'
export EDITOR=vim

###################################
# -- new farol package python -- #
###################################
alias farol_pkg_py="source ${FAROL_SCRIPTS}/farol_new_packages_scripts/farol_create_ros_pkg_py.sh"

#########################
# -- new package c++ -- #
#########################
alias farol_pkg_cpp="source ${FAROL_SCRIPTS}/farol_new_packages_scripts/farol_create_ros_pkg_cpp.sh"

##########################
# -- new package meta -- #
##########################
alias farol_pkg_meta="source ${FAROL_SCRIPTS}/farol_new_packages_scripts/farol_create_ros_pkg_meta.sh"

####################################
# -- new custom bringup package -- #
####################################
alias dsor_pkg_bringup="source ${FAROL_SCRIPTS}/farol_create_bringup_scripts/farol_create_bringup_pkg.sh"

###################################################
# -- add new vehicle to custom bringup package -- #
###################################################
alias dsor_add_vehicle_bringup="source ${FAROL_SCRIPTS}/farol_create_bringup_scripts/farol_add_vehicle_bringup.sh"

######################################################################
# -- Personal configs if hostname file exists (vehicles included) -- #
######################################################################
if [ -f "${ROS_WORKSPACE}/src/medusa_addons/medusa_scripts/system_configurations/medusa_personal_alias/${HOSTNAME}_alias.sh" ]; then
	source ${ROS_WORKSPACE}/src/medusa_addons/medusa_scripts/system_configurations/medusa_personal_alias/${HOSTNAME}_alias.sh
fi