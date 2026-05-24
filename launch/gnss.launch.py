"""Launch u-blox GNSS driver with NavSatFix output for Mr Pinchy."""
import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('mr_pinchy_bringup')
    default_params_file = os.path.join(pkg_dir, 'config', 'gnss.yaml')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='INFO',
    )
    params_file_arg = DeclareLaunchArgument(
        'gnss_params_file',
        default_value=default_params_file,
        description='Path to GNSS parameter YAML file',
    )

    params_file = LaunchConfiguration('gnss_params_file')

    dgnss_container = ComposableNodeContainer(
        name='ublox_dgnss_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                parameters=[params_file],
            ),
        ],
    )

    navsatfix_container = ComposableNodeContainer(
        name='ublox_nav_sat_fix_hp_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp',
            ),
        ],
    )

    return launch.LaunchDescription([
        log_level_arg,
        params_file_arg,
        dgnss_container,
        navsatfix_container,
    ])
