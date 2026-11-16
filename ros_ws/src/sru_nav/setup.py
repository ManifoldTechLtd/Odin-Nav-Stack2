#!/usr/bin/env python3
"""catkin setup for the SRU navigation node (ROS1).

The `sru` directory at the package root is a relative symlink to the single
shared source tree <repo>/sru/src (also used by the ROS2 ament package),
so both distributions execute one code path. setuptools only accepts package
directories inside the project, hence the symlink instead of a package_dir
pointing outside — and the repo must be git-cloned (zip downloads drop
symlinks).
"""
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['sru'],
    package_dir={'': '.'},
)

setup(**setup_args)