"""Launch the STIM300 IMU driver using the Mr Pinchy IMU config."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_arg(context, name):
    return LaunchConfiguration(name).perform(context)


def launch_setup(context, *args, **kwargs):
    parameters = [_launch_arg(context, 'imu_params_file')]

    device = _launch_arg(context, 'device')
    if device:
        parameters.append({'device_name': device})

    sample_rate = _launch_arg(context, 'sample_rate')
    if sample_rate:
        parameters.append({'configure_sample_rate': int(sample_rate)})

    return [
        Node(
            package='stim300',
            executable='stim300_driver',
            name='stim300_driver',
            namespace=_launch_arg(context, 'namespace'),
            parameters=parameters,
            output='screen',
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('mr_pinchy_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'imu.yaml'),
            description='Path to IMU parameter YAML file',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='Serial device override (empty = use imu_params_file)',
        ),
        DeclareLaunchArgument(
            'sample_rate',
            default_value='',
            description='Sensor sample rate override in Hz (empty = use imu_params_file)',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='sam',
            description='Node namespace (the IMU publishes <namespace>/core/imu)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
