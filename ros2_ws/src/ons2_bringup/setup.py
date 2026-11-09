import os
from glob import glob

from setuptools import setup

package_name = 'ons2_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manifold Tech Ltd.',
    maintainer_email='yuhaowen@manifoldtech.cn',
    description='Profile-driven bringup for ONS2 robots.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
)
