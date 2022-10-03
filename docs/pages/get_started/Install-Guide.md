## **1. Ubuntu and ROS installation:**
1. Install Ubuntu 20.04LTS (https://releases.ubuntu.com/20.04)
2. Install ROS 1 Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
3. Add the following Functions and Alias to your `.bashrc` file, to make development easier 🤓

    1.  Create a file to store the latest catkin workspace (if it does not exist) and put in the first line the default name, i.e. catkin_ws
        ```
        if [ ! -f ~/.catkin_ws_config ]; then touch ~/.catkin_ws_config && echo catkin_ws > ~/.catkin_ws_config ;fi
        ```
    2. Set the variable CATKIN_PACKAGE with the workspace in the catkin_ws_config file
        ```
        export CATKIN_PACKAGE=$(head -n 1 ~/.catkin_ws_config)
        ```
    3. Function to update the default catkin workspace variable and store the last setting in the file
        ```
        set_catkin_ws_function() {
            #set CATKIN_PACKAGE according the an input parameter
            export CATKIN_PACKAGE=catkin_ws_$1
            echo CATKIN_PACKAGE = ${CATKIN_PACKAGE}
    
            # save into a hidden file the catkin workspace setting
            echo $CATKIN_PACKAGE > ~/.catkin_ws_config
            source ~/.bashrc
        }
        ```
    4. This is required (to source the ROS and farol files)
        ```
        source /opt/ros/noetic/setup.bash
        export CATKIN_ROOT=${HOME}/<path_to_workspace>
        export ROS_WORKSPACE=${CATKIN_ROOT}/${CATKIN_PACKAGE}
        export FAROL_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname farol_scripts | head -n 1)
        source ${FAROL_SCRIPTS}/farol_easy_alias/farol_permanent_alias/alias.sh
        ```
NOTE: replace `/<path_to_workspace>` with the folder where you put you catkin_ws inside (for example `/dsor`). If you put in your home folder, then this variable should be left empty!

4. Create a catkin_ws directory
```
cd ~
mkdir -p <path_to_workspace>/catkin_ws_farol/src
cd <path_to_workspace>/catkin_ws_farol/src
```

## **2. Downloading the repository:**

Start by cloning the repository with `git clone --recursive https://github.com/dsor-isr/farol`.

If the repository was cloned non-recursively previously, use `git submodule update --init` to clone the necessary submodules.

## **3. Configuring the dependencies:**
Run the installation bash script using
```
cd ~
wget /https://github.com/dsor-isr/farol/blob/main/farol_addons/farol_docker/install_requirements.sh
./install_requirements.sh
rm install_requirements.sh
```

