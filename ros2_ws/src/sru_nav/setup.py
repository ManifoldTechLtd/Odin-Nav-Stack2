"""ament_python build for the ROS2 distribution of the SRU navigation node.

The navigation library (``sru``) is NOT duplicated here: ``sru`` at the
package root is a symlink to the shared source tree (``<repo>/sru/src``)
so the ROS1 and ROS2 distributions always run the same code. Only the ament
build layer, launch file and parameter files live in this package.

Build:
    cd <repo>/ros2_ws && colcon build --packages-select sru_nav
Run:
    ros2 launch sru_nav sru_nav.launch.py
"""

import os
from glob import glob

from setuptools import setup

package_name = 'sru_nav'


def _files(pattern):
    """glob() that tolerates an empty/missing directory (setuptools dislikes []) ."""
    found = sorted(glob(pattern))
    return found


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (os.path.join('share', package_name), ['package.xml']),
    (os.path.join('share', package_name, 'launch'), _files('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), _files('config/*.yaml')),
    (os.path.join('share', package_name, 'test'), _files('test/*.py')),
]
_models = _files('models/*.onnx')
if _models:
    data_files.append((os.path.join('share', package_name, 'models'), _models))

setup(
    name=package_name,
    version='1.0.0',
    # 'sru' resolves through the root-level symlink -> one shared copy.
    # setuptools only accepts package directories inside the project, hence
    # the symlink instead of a package_dir pointing outside.
    package_dir={'': '.'},
    packages=['sru'],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hongle Mo',
    maintainer_email='mohongle@manifoldtech.cn',
    description='ROS2 (Jazzy) distribution of the SRU mapless visual navigation controller.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sru_nav_node = sru.node_main:main',
        ],
    },
)