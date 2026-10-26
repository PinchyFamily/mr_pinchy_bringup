#!/usr/bin/env python3
"""Nucleus DVL, Water Linked sonar and the Nucleus EKF, with a static base_link → sonar_link TF."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vehicle_config import default_secrets_path, load_nucleus_secrets  # noqa: E402


def _nucleus_connect(context):
    nucleus = load_nucleus_secrets(LaunchConfiguration('secrets_file').perform(context))

    # Connect to the Nucleus after a short delay
    return [TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'run',
                    'nucleus_driver_ros2',
                    'connect_tcp',
                    nucleus['host'],
                    nucleus['password'],
                ],
                output='screen',
            )
        ],
    )]


def generate_launch_description():

    sonar_launch = os.path.join(
        get_package_share_directory('waterlinked_sonar_3d15'),
        'launch',
        'sonar_3d15.launch.py',
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'secrets_file',
            default_value=default_secrets_path(),
            description='Path to secrets YAML file (Nucleus host and password in the nucleus section)',
        ),

        # Start the Nucleus driver
        Node(
            package='nucleus_driver_ros2',
            executable='nucleus_node',
            name='nucleus_node',
            output='screen',
        ),

        OpaqueFunction(function=_nucleus_connect),

        # Call the start service once the node is up
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'service',
                        'call',
                        '/nucleus_node/start',
                        'interfaces/srv/Start',
                        '{}',
                    ],
                    output='screen',
                )
            ],
        ),

        # Launch the Water Linked sonar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sonar_launch)
        ),

        # Start the EKF
        Node(
            package='mr_pinchy_bringup',
            executable='nucleus_ekf.py',
            output='screen',
        ),

        # base_link is FRD here (as published by nucleus_ekf.py): sonar 16 cm up, pitched 30° down
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_sonar_tf',
            arguments=[
                '--x', '0.22',
                '--y', '0.0',
                '--z', '-0.16',
                '--roll', '0.0',
                '--pitch', '-0.5235987756',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'sonar_link',
            ],
        ),
    ])
