"""Jetson LiDAR perception service entrypoint, isolated from the HIL RT stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('adas_launch'),
        'config', 'lidar_perception.yaml')
    config = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_config,
            description='Jetson LiDAR perception parameter file'),
        Node(
            package='adas_lidar_perception',
            executable='lidar_perception_node',
            name='lidar_perception',
            output='screen',
            parameters=[config],
        ),
    ])
