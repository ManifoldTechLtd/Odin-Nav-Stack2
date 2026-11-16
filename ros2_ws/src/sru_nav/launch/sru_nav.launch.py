"""ROS2 launch for the SRU navigation controller (Odin1 sensor).

ROS2-specific notes:

1. Parameters are handed over with ``--ros-args --params-file <yaml>`` instead of
   ``<rosparam command="load">``. The node reads its topics through *private*
   parameter lookups, so a ``--remap-from cmd_vel`` would silently do nothing;
   ``--params-file`` is what actually reaches it.
2. ``static_transform_publisher`` takes named flags in ROS2, removing the
   ROS1 positional-argument ambiguity between euler and quaternion forms.

Usage:
    ros2 launch sru_nav sru_nav.launch.py
    ros2 launch sru_nav sru_nav.launch.py use_sim:=true
    ros2 launch sru_nav sru_nav.launch.py launch_static_tf:=false
    ros2 launch sru_nav sru_nav.launch.py \\
        odin1_x:=0.38 odin1_z:=0.05 odin1_pitch:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'sru_nav'


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    default_params = os.path.join(pkg_share, 'config', 'sru_nav_ros2.yaml')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ROS2 parameter file for sru_nav_node.'),
        DeclareLaunchArgument('use_sim', default_value='false',
                              description='Odom twist already in base frame (skip rotation).'),
        DeclareLaunchArgument('launch_static_tf', default_value='true',
                              description='Publish base_link -> odin1_base_link. Disable if the '
                                          'nav stack already publishes this edge (duplicate-TF clash).'),
        # Odin1 mounting (measure on the real platform): x = camera forward of
        # body centre, z = camera height above ground - base_link standing height.
        DeclareLaunchArgument('odin1_x', default_value='0.25'),
        DeclareLaunchArgument('odin1_y', default_value='0.0'),
        DeclareLaunchArgument('odin1_z', default_value='0.18'),
        DeclareLaunchArgument('odin1_roll', default_value='0.0'),
        DeclareLaunchArgument('odin1_pitch', default_value='0.349'),
        DeclareLaunchArgument('odin1_yaw', default_value='0.0'),
        # Model path overrides. Models are NOT bundled with this repository;
        # pass ABSOLUTE paths in production so the loaded policy is unambiguous.
        DeclareLaunchArgument('vae_model_path', default_value='',
                              description='Override VAE encoder onnx path (empty = keep params_file value).'),
        DeclareLaunchArgument('policy_model_path', default_value='',
                              description='Override policy onnx path (empty = keep params_file value).'),
    ]

    def _make_sru_node(context):
        """Build sru_nav_node from the params file.

        --params-file installs the file's values; a later -p wins. Only use_sim
        and the model paths are exposed as launch arguments (runtime flags /
        out-of-tree files); topics live in the params file alone.
        """
        node_args = [
            '--ros-args',
            '--params-file', LaunchConfiguration('params_file').perform(context),
            '-p', 'use_sim:=' + LaunchConfiguration('use_sim').perform(context),
        ]
        vae_model_path = LaunchConfiguration('vae_model_path').perform(context)
        if vae_model_path:
            node_args += ['-p', 'vae_model_path:=' + vae_model_path]
        policy_model_path = LaunchConfiguration('policy_model_path').perform(context)
        if policy_model_path:
            node_args += ['-p', 'policy_model_path:=' + policy_model_path]
        return [Node(
            package=PKG,
            executable='sru_nav_node',
            name='sru_nav_node',
            output='screen',
            emulate_tty=True,
            arguments=node_args,
        )]

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_odin1',
        output='log',
        arguments=[
            '--x', LaunchConfiguration('odin1_x'),
            '--y', LaunchConfiguration('odin1_y'),
            '--z', LaunchConfiguration('odin1_z'),
            '--roll', LaunchConfiguration('odin1_roll'),
            '--pitch', LaunchConfiguration('odin1_pitch'),
            '--yaw', LaunchConfiguration('odin1_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'odin1_base_link',
        ],
        condition=IfCondition(LaunchConfiguration('launch_static_tf')),
    )

    return LaunchDescription(
        args + [static_tf_node, OpaqueFunction(function=_make_sru_node)])
