"""Version-agnostic entry logic for the SRU navigation node.

Shared by the ROS1 catkin entry script and the ROS2 console entry point
(both call ``main`` directly), so there is exactly one implementation of
parameter reading and model lookup.

Package lookup is delegated to ``ros_shim.get_package_path`` (rospkg on ROS1,
ament index on ROS2). Both distributions use the package name ``sru_nav``.
"""

import os
import sys

from sru import constants
from sru import ros_shim as ros
from sru.navigation_policy_node import NavigationPolicyNode

#: Package names to probe for the ``models/`` directory.
PKG_CANDIDATES = ('sru_nav',)


def _package_share_dir(pkg_name):
    """Package share directory on ROS1 (rospkg) or ROS2 (ament index)."""
    return ros.get_package_path(pkg_name)


def _resolve_model_path(param_value, default_basename, pkg_candidates=PKG_CANDIDATES):
    """Resolve a model path.

    Order: explicit absolute path -> ``<package>/models/<basename>`` for each
    candidate package -> the original ``sru-robot-deployment`` sibling layout
    (kept for checkouts that vendor the upstream repo next to this one).
    """
    if param_value and os.path.isabs(param_value) and os.path.exists(param_value):
        return param_value

    candidates = []
    if param_value:
        candidates.append(param_value)
    for pkg_name in pkg_candidates:
        pkg_path = _package_share_dir(pkg_name)
        if not pkg_path:
            continue
        candidates.append(os.path.join(pkg_path, 'models', default_basename))
        # Sibling deployment_policies folder (when sources are checked out together)
        candidates.append(os.path.join(
            pkg_path, '..', '..', 'sru-robot-deployment',
            'rl_nav_controller', 'deployment_policies', default_basename))

    for c in candidates:
        if c and os.path.exists(c):
            return os.path.abspath(c)

    raise FileNotFoundError(
        'Could not locate {} (checked: {})'.format(default_basename, candidates)
    )


def main(pkg_candidates=PKG_CANDIDATES):
    ros.init_node('sru_nav_node', anonymous=False)
    ros.loginfo('SRU node running on {} (ros_shim)'.format(ros.active_ros_version()))

    # --- Topics ---------------------------------------------------------
    depth_topic = ros.get_param('~depth_topic', '/odin1/depth_img_competetion')
    odom_topic = ros.get_param('~odom_topic', '/odin1/odometry_highfreq')
    goal_topic = ros.get_param('~goal_topic', '/sru/goal_pose')
    cmd_vel_topic = ros.get_param('~cmd_vel_topic', '/sru/cmd_vel')

    # --- Models ---------------------------------------------------------
    vae_param = ros.get_param('~vae_model_path', '')
    pol_param = ros.get_param('~policy_model_path', '')
    try:
        vae_model_path = _resolve_model_path(vae_param, 'vae_encoder.onnx', pkg_candidates)
        policy_model_path = _resolve_model_path(pol_param, 'nav_policy.onnx', pkg_candidates)
    except FileNotFoundError as exc:
        ros.logerr(str(exc))
        sys.exit(1)
    ros.loginfo('VAE encoder:   {}'.format(vae_model_path))
    ros.loginfo('Policy model:  {}'.format(policy_model_path))

    # --- Numeric params -------------------------------------------------
    min_depth = ros.get_param('~min_depth', constants.DEFAULT_MIN_DEPTH)
    max_depth = ros.get_param('~max_depth', constants.DEFAULT_MAX_DEPTH)
    control_frequency = ros.get_param(
        '~control_frequency', constants.DEFAULT_CONTROL_FREQUENCY)
    depth_stale_sec = ros.get_param(
        '~depth_stale_sec', constants.DEPTH_STALE_SEC)
    policy_scale = ros.get_param('~policy_scale', list(constants.POLICY_SCALE))
    use_sim = ros.get_param('~use_sim', False)

    if not isinstance(policy_scale, (list, tuple)) or len(policy_scale) != 3:
        ros.logwarn('Invalid ~policy_scale, falling back to {}'.format(
            list(constants.POLICY_SCALE)))
        policy_scale = list(constants.POLICY_SCALE)

    NavigationPolicyNode(
        preprocess_model_path=vae_model_path,
        policy_model_path=policy_model_path,
        depth_topic=depth_topic,
        odom_topic=odom_topic,
        goal_topic=goal_topic,
        cmd_vel_topic=cmd_vel_topic,
        min_depth=min_depth,
        max_depth=max_depth,
        control_frequency=control_frequency,
        policy_scale=policy_scale,
        use_sim=use_sim,
        depth_stale_sec=depth_stale_sec,
    )

    ros.loginfo('SRU navigation node spinning (sim={}).'.format(use_sim))
    ros.spin()
