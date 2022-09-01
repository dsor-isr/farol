#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['cpf_gamma_algorithms', 'cpf_gamma_ros'],
 package_dir={'cpf_gamma_algorithms': 'src/cpf_gamma_algorithms', 'cpf_gamma_ros': 'src/cpf_gamma_ros'}
)

setup(**d)
