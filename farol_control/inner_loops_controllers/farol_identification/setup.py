#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['farol_identification_algorithms', 'farol_identification_ros'],
 package_dir={'farol_identification_algorithms': 'src/farol_identification_algorithms', 'farol_identification_ros': 'src/farol_identification_ros'}
)

setup(**d)
