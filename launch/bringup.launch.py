#!/usr/bin/env python3
"""Full Mr Pinchy vehicle bringup: GNSS, FCU, sonar, DVL, sensor bridges, and static TFs."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vehicle_config import (  # noqa: E402
    _bool_to_launch,
    default_vehicle_config_path,
    load_resolved_config,
    static_transform_node,
)


def _launch_arg(context, name):
    return LaunchConfiguration(name).perform(context)


def launch_setup(context, *args, **kwargs):
    # --- Resolve vehicle config (YAML + profile preset + CLI overrides) ---
    vehicle_config_path = _launch_arg(context, 'vehicle_config')
    if not vehicle_config_path:
        vehicle_config_path = default_vehicle_config_path()

    overrides = {
        'gnss': _launch_arg(context, 'gnss'),
        'sonar': _launch_arg(context, 'sonar'),
        'dvl': _launch_arg(context, 'dvl'),
        'mavros': _launch_arg(context, 'mavros'),
        'nucleus_ip': _launch_arg(context, 'nucleus_ip'),
        'fcu_url': _launch_arg(context, 'fcu_url'),
        'sonar_params_file': _launch_arg(context, 'sonar_params_file'),
    }
    profile = _launch_arg(context, 'profile')
    resolved = load_resolved_config(vehicle_config_path, profile, overrides)
    subsystems = resolved['subsystems']
    network = resolved['network']
    bridges = resolved['bridges']
    extrinsics = resolved['extrinsics']

    pkg_share = get_package_share_directory('mr_pinchy_bringup')
    sonar_pkg = get_package_share_directory('waterlinked_sonar_3d15')
    log_level = _launch_arg(context, 'log_level')

    actions = []

    # --- GNSS: u-blox driver, NTRIP RTK corrections, NavSatFix on /fix ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gnss.launch.py')
        ),
        condition=IfCondition(_bool_to_launch(subsystems['gnss'])),
        launch_arguments={'log_level': log_level}.items(),
    ))

    # --- Sonar: Water Linked 3D15 driver (point cloud + sonar topics) ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sonar_pkg, 'launch', 'sonar_3d15.launch.py')
        ),
        condition=IfCondition(_bool_to_launch(subsystems['sonar'])),
        launch_arguments={
            'params_file': network['sonar_params_file'],
        }.items(),
    ))

    # --- FCU: MAVROS PX4 link (attitude, position, RC, etc. under /mavros/) ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mavros.launch.py')
        ),
        condition=IfCondition(_bool_to_launch(subsystems['mavros'])),
        launch_arguments={
            'fcu_url': network['mavros_fcu_url'],
        }.items(),
    ))

    # --- Static TFs: base_link → sonar, camera, and DVL frames (from pinchy.yaml extrinsics) ---
    base_frame = extrinsics['base_frame']
    for key, node_name in (
        ('sonar', 'base_link_to_sonar'),
        ('camera', 'base_link_to_camera'),
        ('dvl', 'base_link_to_dvl'),
    ):
        ext = extrinsics[key]
        actions.append(static_transform_node(
            node_name,
            base_frame,
            ext['child_frame'],
            ext['xyz'],
            ext['rpy'],
        ))

    # --- DVL: Nucleus driver node, then timed TCP connect and start streaming ---
    dvl_enabled = _bool_to_launch(subsystems['dvl'])
    actions.append(Node(
        package='nucleus_driver_ros2',
        executable='nucleus_node',
        name='nucleus_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(dvl_enabled),
    ))
    actions.append(TimerAction(
        period=float(network['nucleus_connect_delay']),
        condition=IfCondition(dvl_enabled),
        actions=[
            Node(
                package='nucleus_driver_ros2',
                executable='connect_tcp',
                name='nucleus_connect',
                arguments=[network['nucleus_ip'], network['nucleus_vendor']],
                output='screen',
                emulate_tty=True,
            )
        ],
    ))
    actions.append(TimerAction(
        period=float(network['nucleus_start_delay']),
        condition=IfCondition(dvl_enabled),
        actions=[
            Node(
                package='nucleus_driver_ros2',
                executable='start',
                name='nucleus_start',
                output='screen',
                emulate_tty=True,
            )
        ],
    ))

    # --- Sensor bridges: Nucleus/FCU vendor messages → /dvl/*, depth_odom, etc. ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensors_bridge.launch.py')
        ),
        launch_arguments={
            'dvl_type': bridges['dvl_type'],
            'dvl_frame': bridges['dvl_frame'],
            'depth_frame': bridges['depth_frame'],
            'relative_depth': _bool_to_launch(bridges['relative_depth']),
            'depth_topic': bridges['depth_topic'],
            'ned': _bool_to_launch(bridges['ned']),
            'ahrs': _bool_to_launch(bridges['ahrs']),
        }.items(),
    ))

    return actions


def generate_launch_description():
    default_config = default_vehicle_config_path()
    default_sonar = os.path.join(
        get_package_share_directory('mr_pinchy_bringup'), 'config', 'sonar.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_config',
            default_value=default_config,
            description='Path to vehicle YAML config file',
        ),
        DeclareLaunchArgument(
            'profile',
            default_value='full',
            description='Launch preset: full, sensors_only, gnss_only, or sim',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='INFO',
            description='ROS log level for GNSS nodes',
        ),
        DeclareLaunchArgument(
            'gnss',
            default_value='',
            description='Enable GNSS (empty = use profile/config)',
        ),
        DeclareLaunchArgument(
            'sonar',
            default_value='',
            description='Enable sonar (empty = use profile/config)',
        ),
        DeclareLaunchArgument(
            'dvl',
            default_value='',
            description='Enable DVL (empty = use profile/config)',
        ),
        DeclareLaunchArgument(
            'mavros',
            default_value='',
            description='Enable MAVROS (empty = use profile/config)',
        ),
        DeclareLaunchArgument(
            'nucleus_ip',
            default_value='',
            description='Nucleus DVL IP (empty = use vehicle config)',
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='',
            description='MAVROS FCU URL (empty = use vehicle config)',
        ),
        DeclareLaunchArgument(
            'sonar_params_file',
            default_value='',
            description=f'Sonar params YAML (empty = use vehicle config, default {default_sonar})',
        ),
        OpaqueFunction(function=launch_setup),
    ])
