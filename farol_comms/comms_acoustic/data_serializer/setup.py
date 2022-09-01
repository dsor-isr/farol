#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['data_serializer_algorithms', 'data_serializer_ros'],
 package_dir={'data_serializer_algorithms': 'src/data_serializer_algorithms', 'data_serializer_ros': 'src/data_serializer_ros'}
)

setup(**d)
