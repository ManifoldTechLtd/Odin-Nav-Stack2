"""Profile-driven ONS2 ROS 2 bringup: static TF, scan chain, global/local planners, chassis bridge."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_profile import (
    bridge_launch_arguments,
    bridge_launch_file,
    footprint_parameters,
    global_planner_backend,
    global_planner_launch,
    load_profile,
    local_planner_backend,
    local_planner_launch,
    map_planner_parameters,
    neupan_config_path,
    local_planner_config_path,
    pc_to_scan_parameters,
    sensor_mount_config,
)


def _static_tf_node(name, parent, child, x=0.0, y=0.0, z=0.0,
                    qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', f'{x:g}', '--y', f'{y:g}', '--z', f'{z:g}',
            '--qx', f'{qx:g}', '--qy', f'{qy:g}',
            '--qz', f'{qz:g}', '--qw', f'{qw:g}',
            '--frame-id', parent, '--child-frame-id', child,
        ],
        output='screen',
    )


def _include_profile_nodes(context, *args, **kwargs):
    profile_path = LaunchConfiguration('profile').perform(context).strip()
    if not profile_path:
        raise RuntimeError('profile launch argument is required')
    profile = load_profile(profile_path)
    mount = sensor_mount_config(profile)
    half_yaw = 0.5 * mount['yaw']
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)

    map_planner_share = get_package_share_directory('map_planner')
    cloud_filter_mode = (
        LaunchConfiguration('cloud_filter').perform(context).strip().lower() or 'on'
    )
    if cloud_filter_mode not in ('off', 'shadow', 'on'):
        raise RuntimeError(f"cloud_filter must be 'off', 'shadow', or 'on' (got {cloud_filter_mode!r})")

    cloud_filter_nodes = []
    pc_additional_env = None
    nav_cloud_topic = '/odin1/cloud_slam'
    if cloud_filter_mode in ('shadow', 'on'):
        cloud_filter_params_file = os.path.join(
            map_planner_share, 'config', 'odin_cloud_filter.yaml')
        cloud_filter_fastdds_file = os.path.join(
            map_planner_share, 'config', 'odin_cloud_filter_fastdds.xml')
        pc_additional_env = {'FASTRTPS_DEFAULT_PROFILES_FILE': cloud_filter_fastdds_file}
        cloud_filter_nodes.append(Node(
            package='map_planner',
            executable='odin_slam_temporal_filter.py',
            name='odin_cloud_temporal_filter',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            additional_env=pc_additional_env,
            parameters=[cloud_filter_params_file],
        ))
        if cloud_filter_mode == 'on':
            nav_cloud_topic = '/odin1/cloud_filtered'

    pc_params = {
        'target_frame': 'odin1_base_link',
        'qos_overrides./tf.subscription.reliability': 'best_effort',
        'transform_tolerance': 0.1,
        'angle_min': -0.7853,
        'angle_max': 0.7853,
        'angle_increment': 0.017,
        'scan_time': 0.1,
        'range_min': 0.2,
        'range_max': 5.0,
    }
    pc_params.update(pc_to_scan_parameters(profile))
    pc_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_scan',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        additional_env=pc_additional_env,
        parameters=[pc_params],
        remappings=[
            ('cloud_in', nav_cloud_topic),
            ('scan', '/scan_raw'),
        ],
    )

    sliding_map_params_file = os.path.join(
        get_package_share_directory('sliding_map'), 'config', 'sliding_map.yaml')
    sliding_map_node = Node(
        package='sliding_map',
        executable='sliding_map_node',
        name='sliding_map_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[sliding_map_params_file, footprint_parameters(profile)],
    )

    gp_backend = global_planner_backend(
        profile, LaunchConfiguration('global_planner').perform(context).strip())
    planner_actions = []
    if gp_backend == 'map_planner':
        shared_nav_file = os.path.join(
            get_package_share_directory('map_planner'), 'config', 'nav_params.yaml')
        planner_params = load_profile(shared_nav_file) if os.path.isfile(shared_nav_file) else {}
        planner_params.pop('_profile_dir', None)
        planner_params.update(map_planner_parameters(profile))
        planner_actions.append(Node(
            package='map_planner',
            executable='map_planner_node',
            name='map_planner',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[planner_params],
        ))
    else:
        gp_pkg, gp_launch = global_planner_launch(profile, gp_backend)
        if gp_pkg and gp_launch:
            planner_actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory(gp_pkg), 'launch', gp_launch)),
            ))

    lp_backend = local_planner_backend(
        profile, LaunchConfiguration('local_planner').perform(context).strip())
    if lp_backend == 'neupan':
        neupan_cfg = (
            LaunchConfiguration('neupan_config').perform(context).strip()
            or neupan_config_path(profile)
        )
        planner_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('neupan_ros'),
                'launch', 'neupan_ros.launch.py')),
            launch_arguments={
                'config_file': neupan_cfg,
                'navigation_mode': LaunchConfiguration('navigation_mode'),
                'periodic_replan_sec': LaunchConfiguration('periodic_replan_sec'),
            }.items(),
        ))
    elif lp_backend != 'neupan':
        lp_pkg, lp_launch = local_planner_launch(profile, lp_backend)
        if lp_pkg and lp_launch:
            # Optional per-robot overrides (backends.<name>.config in
            # robot.yaml), e.g. robots/sw01/sru/config.yaml -> params_file.
            lp_args = {}
            lp_cfg = local_planner_config_path(profile, lp_backend)
            if lp_cfg:
                lp_args['params_file'] = lp_cfg
            planner_actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory(lp_pkg), 'launch', lp_launch)),
                launch_arguments=lp_args.items(),
            ))

    bridge_pkg, bridge_launch = bridge_launch_file(profile, 'ros2')
    if bridge_pkg and bridge_launch:
        planner_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(bridge_pkg), 'launch', bridge_launch)),
            launch_arguments=bridge_launch_arguments(profile, 'ros2').items(),
        ))

    return [
        _static_tf_node('imu_to_odin1_base', 'imu', 'odin1_base_link'),
        _static_tf_node(
            'odin1_to_base',
            mount['parent'],
            mount['child'],
            x=mount['x'],
            y=mount['y'],
            z=mount['z'],
            qz=qz,
            qw=qw,
        ),
        _static_tf_node(
            'odin1_to_lidar',
            'odin1_base_link',
            'odin1_lidar_link',
            x=0.00347,
            y=0.03447,
            z=0.02174,
        ),
        _static_tf_node(
            'lidar_to_camera',
            'odin1_lidar_link',
            'odin1_camera_link',
            x=0.02055,
            y=0.0004,
            z=-0.00735,
            qx=0.497,
            qy=-0.499,
            qz=0.499,
            qw=0.504,
        ),
        *cloud_filter_nodes,
        pc_to_scan,
        sliding_map_node,
        *planner_actions,
    ]


def generate_launch_description():
    declare_profile = DeclareLaunchArgument(
        'profile',
        description='Path to robots/<robot>/robot.yaml')
    declare_cloud_filter = DeclareLaunchArgument(
        'cloud_filter', default_value='on',
        description='Odin SLAM point-cloud persistence filter mode: on (default) | shadow | off')
    declare_global_planner = DeclareLaunchArgument(
        'global_planner', default_value='',
        description='Optional override for global_planner.backend in robot.yaml')
    declare_local_planner = DeclareLaunchArgument(
        'local_planner', default_value='',
        description='Optional override for local_planner.backend in robot.yaml')
    declare_neupan_config = DeclareLaunchArgument(
        'neupan_config', default_value='',
        description='Optional override for robots/<robot>/neupan/config.yaml')
    declare_navigation_mode = DeclareLaunchArgument(
        'navigation_mode', default_value='replan_periodic',
        description='NeuPAN/mission navigation mode')
    declare_periodic = DeclareLaunchArgument(
        'periodic_replan_sec', default_value='2.0',
        description='Periodic replan interval in seconds')

    scan_filter = Node(
        package='map_planner',
        executable='scan_temporal_filter.py',
        name='scan_temporal_filter',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{'enable': False}],
    )

    profile_nodes = OpaqueFunction(function=_include_profile_nodes)

    return LaunchDescription([
        declare_profile,
        declare_cloud_filter,
        declare_global_planner,
        declare_local_planner,
        declare_neupan_config,
        declare_navigation_mode,
        declare_periodic,
        profile_nodes,
        scan_filter,
    ])
