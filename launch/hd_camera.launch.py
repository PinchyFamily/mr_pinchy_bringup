"""Launch the exploreHD camera: H.264 packets for recording, plus an on-demand decoded feed."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mr_pinchy_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'hd_camera.yaml'),
            description='Parameter YAML for the hd_camera_h264 and hd_camera_decoder nodes',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='hd_camera',
            description='Node namespace (publishes <namespace>/h264 and <namespace>/image_decoded)',
        ),
        DeclareLaunchArgument(
            'decode',
            default_value='true',
            description='Start the decoder node (idle, no CPU, until image_decoded has a subscriber)',
        ),
        DeclareLaunchArgument(
            'web_server',
            default_value='true',
            description='Start a web_video_server (false if one is already running)',
        ),
        DeclareLaunchArgument(
            'address',
            default_value='0.0.0.0',
            description='web_video_server bind address (0.0.0.0 = all interfaces, incl. Tailscale)',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8081',
            description='web_video_server HTTP port (3d_camera.launch.py uses 8080)',
        ),
        Node(
            package='mr_pinchy_bringup',
            executable='hd_camera_h264.py',
            name='hd_camera_h264',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='mr_pinchy_bringup',
            executable='hd_camera_decoder.py',
            name='hd_camera_decoder',
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration('decode')),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='hd_web_video_server',
            condition=IfCondition(LaunchConfiguration('web_server')),
            parameters=[{
                'address': LaunchConfiguration('address'),
                'port': LaunchConfiguration('port'),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
