"""Read robots/<robot>/robot.yaml and map robot-specific values to launch args."""

import sys
from pathlib import Path
import yaml


def load_profile(path) -> dict:
    resolved = Path(path).expanduser().resolve()
    with resolved.open(encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    data['_profile_dir'] = str(resolved.parent)
    return data


def neupan_config_path(profile: dict) -> str:
    rel = profile.get('local_planner', {}).get('backends', {}).get('neupan', {}).get('config', '')
    return str((Path(profile['_profile_dir']) / rel).resolve()) if rel else ''


def local_planner_backend(profile: dict, override: str = '') -> str:
    return (override or profile.get('local_planner', {}).get('backend', 'neupan')).strip()


def local_planner_mode(profile: dict, backend: str = '') -> str:
    b = local_planner_backend(profile, backend)
    spec = profile.get('local_planner', {}).get('backends', {}).get(b, {})
    mode = str(spec.get('interface', {}).get('mode', '')).strip()
    if mode in ('action', 'goal_pose'):
        return mode
    return 'goal_pose' if b in ('sru', 'goal_pose') else 'action'


def local_planner_launch(profile: dict, backend: str = '') -> tuple:
    b = local_planner_backend(profile, backend)
    spec = profile.get('local_planner', {}).get('backends', {}).get(b, {})
    return str(spec.get('package', '')).strip(), str(spec.get('launch', '')).strip()


def global_planner_backend(profile: dict, override: str = '') -> str:
    return (override or profile.get('global_planner', {}).get('backend', 'map_planner')).strip()


def global_planner_launch(profile: dict, backend: str = '') -> tuple:
    b = global_planner_backend(profile, backend)
    spec = profile.get('global_planner', {}).get('backends', {}).get(b, {})
    return str(spec.get('package', '')).strip(), str(spec.get('launch', '')).strip()


def sensor_mount_config(profile: dict) -> dict:
    m = profile['geometry']['sensor_mount']
    return {
        'parent': str(m['parent']),
        'child': str(m['child']),
        'x': float(m['x']),
        'y': float(m['y']),
        'z': float(m['z']),
        'yaw': float(m['yaw']),
    }


def footprint_parameters(profile: dict) -> dict:
    fp = profile['geometry']['footprint']
    is_poly = fp['mode'] == 'polygon'
    return {
        'use_footprint_polygon': is_poly,
        'footprint_radius_m': float(fp.get('radius', 0.35)),
        'footprint_length_m': float(fp.get('length', 0.0)),
        'footprint_width_m': float(fp.get('width', 0.0)),
        'footprint_margin_m': float(fp.get('margin', 0.10)),
    }


def pc_to_scan_parameters(profile: dict) -> dict:
    nav = profile.get('navigation', {})
    return {k: float(nav[k]) for k in ('min_height', 'max_height') if k in nav}


def map_planner_parameters(profile: dict) -> dict:
    nav = profile.get('navigation', {})
    cast = {
        'inflation_radius': float,
        'obstacle_cost_safe_distance': float,
        'obstacle_cost_weight': float,
        'line_of_sight_safe_distance': float,
        'smoothing_iterations': int,
        'smoothing_max_deviation_m': float,
    }
    return {k: fn(nav[k]) for k, fn in cast.items() if k in nav}


def bridge_launch_file(profile: dict, runtime: str = 'ros2') -> tuple:
    cfg = profile.get('bridge', {}).get(runtime, {})
    return str(cfg.get('package', '')).strip(), str(cfg.get('launch', '')).strip()


def bridge_launch_arguments(profile: dict, runtime: str = 'ros2') -> dict:
    bridge = profile.get('bridge', {})
    topic = bridge.get(runtime, {}).get(
        'cmd_vel_topic', bridge.get('common', {}).get('cmd_vel_topic'))
    return {'cmd_vel_topic': str(topic)} if topic else {}


def mission_interface_arguments(profile: dict, lp_backend: str = '', gp_backend: str = '') -> dict:
    lb = local_planner_backend(profile, lp_backend)
    gb = global_planner_backend(profile, gp_backend)
    lp_iface = profile.get('local_planner', {}).get('backends', {}).get(lb, {}).get('interface', {})
    gp_spec = profile.get('global_planner', {}).get('backends', {}).get(gb, {})
    args = {}
    if gp_spec.get('plan_service'):
        args['plan_service'] = str(gp_spec['plan_service'])
    for yaml_key, arg_key in (
        ('nav_path_action', 'nav_path_action'),
        ('stop_topic', 'stop_topic'),
        ('goal_topic', 'goal_topic'),
        ('enable_topic', 'enable_topic'),
        ('cmd_vel_topic', 'planner_cmd_vel_topic'),
    ):
        if lp_iface.get(yaml_key):
            args[arg_key] = str(lp_iface[yaml_key])
    return args


def ros1_bringup_arguments(profile: dict, lp_backend: str = '', gp_backend: str = '') -> dict:
    m = sensor_mount_config(profile)
    fp = footprint_parameters(profile)
    lp_pkg, lp_launch = local_planner_launch(profile, lp_backend)
    gp_pkg, gp_launch = global_planner_launch(profile, gp_backend)
    br_pkg, br_launch = bridge_launch_file(profile, 'ros1')
    args = {
        'mount_x': f'{m["x"]:g}',
        'mount_y': f'{m["y"]:g}',
        'mount_z': f'{m["z"]:g}',
        'mount_yaw': f'{m["yaw"]:g}',
        'use_footprint_polygon': 'true' if fp['use_footprint_polygon'] else 'false',
        'footprint_radius_m': f'{fp["footprint_radius_m"]}',
        'footprint_length_m': f'{fp["footprint_length_m"]}',
        'footprint_width_m': f'{fp["footprint_width_m"]}',
        'footprint_margin_m': f'{fp["footprint_margin_m"]}',
    }
    if br_pkg and br_launch:
        args['bridge_pkg'] = br_pkg
        args['bridge_launch'] = br_launch
    br_args = bridge_launch_arguments(profile, 'ros1')
    if 'cmd_vel_topic' in br_args:
        args['bridge_cmd_vel_topic'] = br_args['cmd_vel_topic']
    if lp_pkg and lp_launch:
        args['local_planner_pkg'] = lp_pkg
        args['local_planner_launch'] = lp_launch
    if gp_pkg and gp_launch:
        args['global_planner_pkg'] = gp_pkg
        args['global_planner_launch'] = gp_launch
    for k, v in {**pc_to_scan_parameters(profile), **map_planner_parameters(profile)}.items():
        args[k] = f'{v}'
    return args


if __name__ == '__main__':
    if len(sys.argv) in (3, 4, 5) and sys.argv[1] == '--ros1-bringup-args':
        prof = load_profile(sys.argv[2])
        lb = sys.argv[3] if len(sys.argv) >= 4 else ''
        gb = sys.argv[4] if len(sys.argv) >= 5 else ''
        kv = ros1_bringup_arguments(prof, lb, gb)
        print(' '.join(f'{k}:={v}' for k, v in kv.items()))
    elif len(sys.argv) == 3 and sys.argv[1] == '--neupan-config':
        print(neupan_config_path(load_profile(sys.argv[2])))
    elif len(sys.argv) == 3 and sys.argv[1] == '--local-planner':
        print(local_planner_backend(load_profile(sys.argv[2])))
    elif len(sys.argv) == 3 and sys.argv[1] == '--global-planner':
        print(global_planner_backend(load_profile(sys.argv[2])))
    elif len(sys.argv) in (3, 4) and sys.argv[1] == '--local-planner-mode':
        prof = load_profile(sys.argv[2])
        b = sys.argv[3] if len(sys.argv) == 4 else ''
        print(local_planner_mode(prof, b))
    elif len(sys.argv) in (3, 4, 5) and sys.argv[1] == '--mission-args':
        prof = load_profile(sys.argv[2])
        lb = sys.argv[3] if len(sys.argv) >= 4 else ''
        gb = sys.argv[4] if len(sys.argv) >= 5 else ''
        kv = mission_interface_arguments(prof, lb, gb)
        print(' '.join(f'{k}:={v}' for k, v in kv.items()))
    else:
        raise SystemExit(
            'Usage: robot_profile.py (--ros1-bringup-args|--mission-args|--neupan-config|--local-planner|--global-planner|--local-planner-mode) <profile.yaml>')
