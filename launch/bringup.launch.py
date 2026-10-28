#!/usr/bin/env python3
"""Full Mr Pinchy vehicle bringup: GNSS, FCU, sonar, DVL, camera, sensor bridges, and robot description."""
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
    default_secrets_path,
    default_vehicle_config_path,
    load_nucleus_secrets,
    load_resolved_config,
    load_vehicle_config,
    robot_description,
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
        'camera': _launch_arg(context, 'camera'),
        'nucleus_ip': _launch_arg(context, 'nucleus_ip'),
        'sonar_ip': _launch_arg(context, 'sonar_ip'),
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
    log_level = _launch_arg(context, 'log_level')
    secrets_file = _launch_arg(context, 'secrets_file')

    actions = []

    # --- GNSS: u-blox driver, NTRIP RTK corrections, NavSatFix on /fix ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gnss.launch.py')
        ),
        condition=IfCondition(_bool_to_launch(subsystems['gnss'])),
        launch_arguments={'log_level': log_level, 'secrets_file': secrets_file}.items(),
    ))

    # --- Sonar: Water Linked 3D15 driver (params from pinchy.yaml sonar section) ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sonar.launch.py')
        ),
        condition=IfCondition(_bool_to_launch(subsystems['sonar'])),
        launch_arguments={
            'vehicle_config': vehicle_config_path,
            'sonar_params_file': _launch_arg(context, 'sonar_params_file'),
            'sonar_ip': _launch_arg(context, 'sonar_ip'),
        }.items(),
    ))

    # --- Camera: auxiliary exploreHD via usb_cam (params from config/hd_camera.yaml) ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hd_camera.launch.py')
        ),
        condition=IfCondition(_bool_to_launch(subsystems['camera'])),
        launch_arguments={'web_server': 'false'}.items(),
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

    # --- Robot description: fixed joints base_link → sensor frames (from pinchy.yaml extrinsics) ---
    robot_name = (load_vehicle_config(vehicle_config_path).get('vehicle') or {}).get('name', 'pinchy')
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description(extrinsics, robot_name)}],
        output='screen',
    ))

    # --- DVL: Nucleus driver node, then timed TCP connect and start streaming ---
    dvl_enabled = _bool_to_launch(subsystems['dvl'])
    # Host and password come from secrets.yaml; nucleus_ip:= still overrides the host.
    nucleus = {'host': '', 'password': ''}
    if subsystems['dvl']:
        nucleus = load_nucleus_secrets(secrets_file)
    nucleus_host = network.get('nucleus_ip') or nucleus['host']
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
                arguments=[nucleus_host, nucleus['password']],
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_config',
            default_value=default_config,
            description='Path to vehicle YAML config file',
        ),
        DeclareLaunchArgument(
            'secrets_file',
            default_value=default_secrets_path(),
            description='Path to secrets YAML file (NTRIP and Nucleus credentials)',
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
            'camera',
            default_value='',
            description='Enable auxiliary exploreHD camera (empty = use profile/config)',
        ),
        DeclareLaunchArgument(
            'nucleus_ip',
            default_value='',
            description='Nucleus DVL IP (empty = use nucleus.host from secrets file)',
        ),
        DeclareLaunchArgument(
            'sonar_ip',
            default_value='',
            description='Water Linked sonar IP (empty = use vehicle config)',
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='',
            description='MAVROS FCU URL (empty = use vehicle config)',
        ),
        DeclareLaunchArgument(
            'sonar_params_file',
            default_value='',
            description='Optional sonar params YAML override (empty = use pinchy.yaml sonar section)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
