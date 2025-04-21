# Farol Base
This repository holds the Farol Base code stack for underwater marine vehicles of DSOR-ISR (Dynamical Systems for Ocean Robotics - Institute for System Robotics). It contains the base of the control and navigation stack found in the DSOR class of marine vehicles(Medusa AUV, BlueRov2, Delfim Catamaran).

[![Build Status](https://ci.dsor.isr.tecnico.ulisboa.pt/buildStatus/icon?job=GitHub+DSOR%2Ffarol%2Fmain)](https://ci.dsor.isr.tecnico.ulisboa.pt/job/GitHub%20DSOR/job/farol/job/main/)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/dsor-isr/farol/main)
![GitHub contributors](https://img.shields.io/github/contributors/dsor-isr/farol)
[![GitHub issues](https://img.shields.io/github/issues/dsor-isr/farol)](https://github.com/dsor-isr/farol/issues)
[![GitHub forks](https://img.shields.io/github/forks/dsor-isr/farol)](https://github.com/dsor-isr/farol/network)
[![GitHub stars](https://img.shields.io/github/stars/dsor-isr/farol)](https://github.com/dsor-isr/farol/stargazers)
[![License](https://img.shields.io/github/license/dsor-isr/farol?color=blue)](https://github.com/dsor-isr/farol/blob/main/LICENSE)

### Requirements
This code stack was developed with ROS1 in mind. In order to use, you are required to have:
- Ubuntu 20.04LTS (64-bit)
- ROS1 Noetic
- Python 3

### Installation
- Clone this repository and its submodules to the catkin workspace:
```bash
git clone --recursive git@github.com:dsor-isr/farol.git
```

- Run the installation script (note: you will require administrator priviledges)
```bash
roscd farol/farol_addons/farol_docker/
chmod u+x install_requirements.sh
./install_requirements.sh
rm install_requirements.sh
```

### Using Farol Scripts and Alias
In order to make use of the scripts and alias developed to make the development of code easier, please add the following lines to your ~/.bashrc file.
NOTE: replace '/<path_to_workspace>' with the folder where you put you catkin_ws inside. If you put in your home folder, then this variable should be left empty!
```bash
# Function to change between different catkin workspaces on the fly - this is not compulsory, but it is a nice addition 🤓

# Create a file to store the latest catkin workspace (if it does not exist) and put in the first line the default name, i.e. catkin_ws
if [ ! -f ~/.catkin_ws_config ]; then touch ~/.catkin_ws_config && echo catkin_ws > ~/.catkin_ws_config ;fi

# Set the variable CATKIN_PACKAGE with the workspace in the catkin_ws_config file
export CATKIN_PACKAGE=$(head -n 1 ~/.catkin_ws_config)

# Function to update the default catkin workspace variable and store the last setting in the file
set_catkin_ws_function() {
    #set CATKIN_PACKAGE according the an input parameter
    export CATKIN_PACKAGE=catkin_ws_$1
    echo CATKIN_PACKAGE = ${CATKIN_PACKAGE}
    
    # save into a hidden file the catkin workspace setting
    echo $CATKIN_PACKAGE > ~/.catkin_ws_config
    source ~/.bashrc
}

# This is required (to source the ROS and farol files)
source /opt/ros/noetic/setup.bash
export CATKIN_ROOT=${HOME}/<path_to_workspace>
export ROS_WORKSPACE=${CATKIN_ROOT}/${CATKIN_PACKAGE}
export FAROL_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname farol_scripts | head -n 1)
source ${FAROL_SCRIPTS}/farol_easy_alias/farol_permanent_alias/alias.sh
```

### Compile the code
- Compile the code
```bash
cd ~/<path_to_workspace>/<catkin_ws>/
catkin build
```

### Citation
If you use Farol in a scientific publication, please cite:
```
TODO
```

### Documentation
https://dsor-isr.github.io/farol/

### Active Developers
- Eduardo Cunha <eduardo.m.a.cunha@tecnico.ulisboa.pt>
- Ravi Regalo <ravi.regalo@tecnico.ulisboa.pt>
- David Cabecinhas <dcabecinhas@isr.tecnico.ulisboa.pt>

### Previous Contributors
- João Quintas <jquintas@gmail.com>
- David Souto <david.souto@tecnico.ulisboa.pt>
- Francisco Branco <francisco.branco@tecnico.ulisboa.pt>
- Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
- Francisco Rego <ffcrego@gmail.com>
- André Potes
- João Cruz
- Hung Tuan
- Shubham Garg
- Jorge Ribeiro
- Miguel Ribeiro
- Henrique Silva
- João Botelho
- Filipa Almeida

### Omnipresent
- Prof. António Pascoal
- Prof. Carlos Silvestre
- Prof. Rita Cunha
- Prof. Bruno Guerreiro
- Prof. Pedro Batista
- Luís Sebastião
- Manuel Rufino
- Pedro Góis
- Helena Santana

### License
Farol is open-sourced under the MIT license. See the [LICENSE](LICENSE) file for details.
