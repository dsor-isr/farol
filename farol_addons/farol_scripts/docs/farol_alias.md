# FAROL Useful alias

## Description
Set of useful aliases to facilitate the use of FAROL stack. Also sources some bash scripts to make your life easier in terminal, please see [FAROL bash scripts](./bash_scripts.md). 

**Note:** You can add your own personal alias, by creating/placing a ${HOSTNAME}_alias.sh file in the following folder inside your catkin workspace:
```
source ${ROS_WORKSPACE}/src/medusa_addons/medusa_scripts/system_configurations/medusa_personal_alias/${HOSTNAME}_alias.sh
```

This part basically checks if a file with your **hostname -> ${HOSTNAME}_alias.sh** exists at the folder *medusa_personal_alias*. So if you want to add some personal alias do the following:

```
Imagine that your hostname is awesome

At the mentioned folder create the following file:
touch awesome_alias.sh

After this edit with your editor of choice and add as many alias as you want.
```

The content of the awesome_alias.sh can be something like this:
```
alias mvehicle1='ssh name@vehicleHostname1'  
alias mvehicle2='ssh name@vehicleHostname2'  
alias mvehicle3='ssh name@vehicleHostname3'  
alias mvehicle4='ssh name@vehicleHostname4'  
alias mvehicle5='ssh name@vehicleHostname5'  
```

## Relevant of aliases

| alias                         | command                                                                           | purpose                                                                                                                |
| -----                         | --------                                                                          | -------                                                                                                                |
| kill_all_ros_nodes            | sudo pkill -f ros                                                                 | kill all ros nodes                                                                                                     |
| rviz                          | rosrun rviz rviz                                                                  | open rviz                                                                                                              |
| tf_view_frames                | cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &             | view tf frames                                                                                                         |
| clean_ros_logs                | rosclean purge -y                                                                 | clean ros logs                                                                                                         |
| farol_cb                      | roscd; catkin build; cd $OLDPWD                                                   | build the entire catkin workspace (you need to be somewhere inside your catkin workspace)                              |
| farol_cbt                     | catkin build --this                                                               | build one pkg (you need to be somewhere inside your ros pkg)                                                           |
| farol_cb_vim                  | roscd; bash ${FAROL_SCRIPTS}/farol_scripts_for_bash/farol_build_vim.bash          | same as farol_cb but for vim users using coc                                                                           |
| cap                           | pygmentize -g                                                                     | replace cat with python-pygments to cat with colors                                                                    |
| ..                            | cd .. && ls                                                                       | going back one directory and showing files convenient alias                                                            |
| m                             | wmctrl -r :ACTIVE: -b toggle,maximized_vert,maximized_horz                        | toggle terminal from restored to maximized                                                                             |
| poweroff                      | sudo shutdown -h now                                                              | shutdown pc                                                                                                            |
| pcdown                        | sudo shutdown -P now                                                              | different way to shutdown pc                                                                                           |
| pcrestart                     | sudo shutdown -r now                                                              | restart pc                                                                                                             |
| please_fiic                   | sudo $(history -p !!)' # run last command as sudo                                 | sudo last command                                                                                                      |
| S                             | source ${HOME}/.bashrc                                                            | source bashrc                                                                                                          |
| remove_endline_spaces         | sed -i 's/\s*$//'                                                                 | remove automatically spaces at the end of files, needs the file as argument at the end, i.e. remove_spaces my_file.txt |
| clean_temp_files              | find . -name "*~" -type f -exec /bin/rm -fv -- {} +                               | to clean temp files *.~ recursively                                                                                    |
| farol_change_inners_gains     | bash ${FAROL_SCRIPTS}/farol_scripts_for_bash/change_inner_forces_gains.bash       | executes a bash scripts that calls a service to change inner loops gains                                               |
| farol_change_pfollowing_gains | bash ${FAROL_SCRIPTS}/farol_scripts_for_bash/change_pfollowing_gains.bash         | executes a bash scripts that calls a service to change outer loops gains                                               |
| farol_pkg_cpp                 | source ${FAROL_SCRIPTS}/farol_new_packages_scripts/farol_create_ros_pkg_cpp.sh    | new package c++                                                                                                        |
| farol_pkg_py                  | source ${FAROL_SCRIPTS}/farol_new_packages_scripts/farol_create_ros_pkg_py.sh     | new package python                                                                                                     |
| farol_pkg_meta                | source ${FAROL_SCRIPTS}/farol_new_packages_scripts/farol_create_ros_pkg_meta.sh   | new metapackage                                                                                                        |
| dsor_pkg_bringup              | source ${FAROL_SCRIPTS}/farol_create_bringup_scripts/farol_create_bringup_pkg.sh  | new custom bringup package                                                                                             |
| dsor_add_vehicle_bringup      | source ${FAROL_SCRIPTS}/farol_create_bringup_scripts/farol_add_vehicle_bringup.sh | add new vehicle to custom bringup packag                                                                               |
| mgit_pull                     | git pull && git submodule update --init --recursive                               | pull and update submodules                                                                                             |
| mgit_status                   | git status && git submodule status --recursive                                    | Get the status of the repository and the version of the submodules                                                     |
| mgit_push_tag                 | git push origin --tags                                                            | Push the local tags to the remote repository                                                                           |
| mgit_describe                 | git describe                                                                      | Get a general description of the repository                                                                            |
| mgit_log                      | git log                                                                           | Get the git log with commits                                                                                           |
