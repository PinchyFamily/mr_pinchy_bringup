"""Launch the RealSense D435i 3D camera and a web_video_server for viewing it from any browser."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    realsense_share = get_package_share_directory('realsense2_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'realsense',
            default_value='true',
            description='Start the RealSense node (false if it is already running)',
        ),
        DeclareLaunchArgument(
            'color_profile',
            default_value='640x480x15',
            description='RealSense colour stream WxHxFPS (kept small for MJPEG streaming)',
        ),
        DeclareLaunchArgument(
            'depth_profile',
            default_value='640x480x15',
            description='RealSense depth stream WxHxFPS',
        ),
        DeclareLaunchArgument(
            'address',
            default_value='0.0.0.0',
            description='web_video_server bind address (0.0.0.0 = all interfaces, incl. Tailscale)',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='web_video_server HTTP port',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_share, 'launch', 'rs_launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('realsense')),
            launch_arguments={
                'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
                'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
                'colorizer.enable': 'true',
            }.items(),
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'address': LaunchConfiguration('address'),
                'port': LaunchConfiguration('port'),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
