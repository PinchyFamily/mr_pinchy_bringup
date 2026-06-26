"""Launch Water Linked 3D15 sonar driver using Mr Pinchy vehicle config."""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vehicle_config import (  # noqa: E402
    default_vehicle_config_path,
    load_resolved_config,
)


def _launch_arg(context, name):
    return LaunchConfiguration(name).perform(context)


def launch_setup(context, *args, **kwargs):
    vehicle_config_path = _launch_arg(context, 'vehicle_config')
    if not vehicle_config_path:
        vehicle_config_path = default_vehicle_config_path()

    overrides = {
        'sonar_params_file': _launch_arg(context, 'sonar_params_file'),
        'sonar_ip': _launch_arg(context, 'sonar_ip'),
    }
    resolved = load_resolved_config(vehicle_config_path, '', overrides)

    return [
        Node(
            package='waterlinked_sonar_3d15',
            executable='sonar_node',
            name='sonar_node',
            namespace=_launch_arg(context, 'namespace'),
            parameters=[resolved['sonar']],
            output='screen',
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_config',
            default_value=default_vehicle_config_path(),
            description='Path to vehicle YAML config file',
        ),
        DeclareLaunchArgument(
            'sonar_params_file',
            default_value='',
            description='Optional sonar params YAML override (sonar_node/ros__parameters format)',
        ),
        DeclareLaunchArgument(
            'sonar_ip',
            default_value='',
            description='Sonar IP override (empty = use vehicle config)',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Node namespace',
        ),
        OpaqueFunction(function=launch_setup),
    ])
