"""Launch u-blox GNSS driver with NTRIP RTK corrections and NavSatFix output."""
import os

import launch
import yaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

NTRIP_SECRET_KEYS = ('host', 'port', 'mountpoint', 'use_https', 'username', 'password')


def load_ntrip_secrets(path):
    """Return NTRIP parameters from secrets.yaml.

    Supports the consolidated format (top-level ``ntrip:`` section) and the
    older ROS params format (``ntrip_client: ros__parameters:``).
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f'secrets file not found: {path} '
            '(copy config/secrets.yaml.example to config/secrets.yaml)')
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if 'ntrip' in data:
        section = data['ntrip']
    else:
        section = data.get('ntrip_client', {}).get('ros__parameters', {})
    return {k: v for k, v in section.items() if k in NTRIP_SECRET_KEYS}


def _ntrip_node(context):
    secrets_file = LaunchConfiguration('secrets_file').perform(context)
    return [Node(
        package='mr_pinchy_bringup',
        executable='ntrip_client_py.py',
        name='ntrip_client',
        parameters=[LaunchConfiguration('gnss_params_file'), load_ntrip_secrets(secrets_file)],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )]


def generate_launch_description():
    pkg_dir = get_package_share_directory('mr_pinchy_bringup')
    default_params_file = os.path.join(pkg_dir, 'config', 'gnss.yaml')
    default_secrets_file = os.path.join(pkg_dir, 'config', 'secrets.yaml')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='INFO',
    )
    params_file_arg = DeclareLaunchArgument(
        'gnss_params_file',
        default_value=default_params_file,
        description='Path to GNSS parameter YAML file',
    )
    secrets_file_arg = DeclareLaunchArgument(
        'secrets_file',
        default_value=default_secrets_file,
        description='Path to secrets YAML file (NTRIP credentials in the ntrip section)',
    )

    params_file = LaunchConfiguration('gnss_params_file')

    dgnss_node = Node(
        package='ublox_dgnss_node',
        executable='ublox_dgnss_node',
        name='ublox_dgnss',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    navsatfix_node = Node(
        package='ublox_nav_sat_fix_hp_node',
        executable='ublox_nav_sat_fix_hp',
        name='ublox_nav_sat_fix_hp',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return launch.LaunchDescription([
        log_level_arg,
        params_file_arg,
        secrets_file_arg,
        dgnss_node,
        OpaqueFunction(function=_ntrip_node),
        navsatfix_node,
    ])
