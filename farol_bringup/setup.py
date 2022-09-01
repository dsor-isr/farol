#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['farol_bringup_algorithms', 'farol_bringup_ros'],
 package_dir={'farol_bringup_algorithms': 'src/farol_bringup_algorithms', 'farol_bringup_ros': 'src/farol_bringup_ros'}
)

setup(**d)
