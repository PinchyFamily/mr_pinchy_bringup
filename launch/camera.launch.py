"""Launch usb_cam for the auxiliary DWE exploreHD camera using Mr Pinchy vehicle config."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vehicle_config import (  # noqa: E402
    default_vehicle_config_path,
    load_vehicle_config,
    resolve_extrinsics,
)


def _launch_arg(context, name):
    return LaunchConfiguration(name).perform(context)


def launch_setup(context, *args, **kwargs):
    vehicle_config_path = _launch_arg(context, 'vehicle_config')
    if not vehicle_config_path:
        vehicle_config_path = default_vehicle_config_path()

    # Match the base_link → camera static TF published by bringup.launch.py.
    extrinsics = resolve_extrinsics(load_vehicle_config(vehicle_config_path))

    return [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace=_launch_arg(context, 'camera_name'),
            parameters=[
                _launch_arg(context, 'camera_params_file'),
                {'frame_id': extrinsics['camera']['child_frame']},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('mr_pinchy_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_config',
            default_value=default_vehicle_config_path(),
            description='Path to vehicle YAML config file',
        ),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=os.path.join(pkg_share, 'config', 'camera.yaml'),
            description='usb_cam parameter file',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='explorehd',
            description='Camera namespace (topics under /<camera_name>/)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
