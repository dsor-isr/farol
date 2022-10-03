# FAROL 

[![Build Status](https://ci.dsor.isr.tecnico.ulisboa.pt/buildStatus/icon?job=GitHub+DSOR%2Ffarol%2Fmain)](https://ci.dsor.isr.tecnico.ulisboa.pt/job/GitHub%20DSOR/job/farol/job/main/)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/dsor-isr/farol/main)
![GitHub contributors](https://img.shields.io/github/contributors/dsor-isr/farol)
[![GitHub issues](https://img.shields.io/github/issues/dsor-isr/farol)](https://github.com/dsor-isr/farol/issues)
[![GitHub forks](https://img.shields.io/github/forks/dsor-isr/farol)](https://github.com/dsor-isr/farol/network)
[![GitHub stars](https://img.shields.io/github/stars/dsor-isr/farol)](https://github.com/dsor-isr/farol/stargazers)
[![License](https://img.shields.io/github/license/dsor-isr/farol?color=blue)](https://github.com/dsor-isr/farol/blob/main/LICENSE)

## Introduction

FAROL (Free Autonomous Robots for Observations and Labelling) is the core software of NetMARSys Toolchain developed at DSOR - ISR (Dynamical Systems for Ocean Robotics - Institute for System Robotics) research group. It is used for simulations and field trials of actual marine vehicles (ASVs, AUVs and ROVs). It comprises a set of ROS packages written in Python and C++ together with external dependencies, please see [Repository Structure](https://dsor-isr.github.io/farol/pages/get_started/Repository-Structure/) for detailed information.

A simple dynamic model simulator is included with FAROL. It is also possible to connect FAROL to a Gazebo simulator with more advanced sensor models, actuators and dynamics. For this check out our repositories
[Farol Gazebo](https://github.com/dsor-isr/farol_gazebo). For a quick setup please check [DSOR Simulation](https://github.com/dsor-isr/dsor_simulation) repository.

Currently FAROL acts as the pivotal piece in the control, navigation and communications of the following DSOR marine vehicles:

* [Medusa](http://dsor.isr.ist.utl.pt/vehicles/medusa/)
* [BlueRov2](https://bluerobotics.com/store/rov/bluerov2/)
* [Delfim](https://welcome.isr.tecnico.ulisboa.pt/wp-content/uploads/2015/05/1501_MED06_DELFIM.pdf)


## Requirements
This code stack was developed with ROS1 in mind. In order to use it, you are required to have:

* Ubuntu 20.04LTS (64-bit)
* ROS1 Noetic
* Python 3
