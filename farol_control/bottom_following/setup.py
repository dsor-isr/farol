#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['bottom_following_algorithms', 'bottom_following_ros'],
 package_dir={'bottom_following_algorithms': 'src/bottom_following_algorithms', 'bottom_following_ros': 'src/bottom_following_ros'}
)

setup(**d)
