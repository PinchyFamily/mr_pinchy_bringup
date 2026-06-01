"""Launch u-blox GNSS driver with NTRIP RTK corrections and NavSatFix output."""
import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
        description='Path to secrets YAML file with NTRIP credentials',
    )

    params_file = LaunchConfiguration('gnss_params_file')
    secrets_file = LaunchConfiguration('secrets_file')

    dgnss_node = Node(
        package='ublox_dgnss_node',
        executable='ublox_dgnss_node',
        name='ublox_dgnss',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    ntrip_node = Node(
        package='mr_pinchy_bringup',
        executable='ntrip_client_py.py',
        name='ntrip_client',
        parameters=[secrets_file],
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
        ntrip_node,
        navsatfix_node,
    ])
