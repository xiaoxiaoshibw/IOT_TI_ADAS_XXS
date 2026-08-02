"""Launch the CARLA OpenDRIVE map provider without changing the HIL stack."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('adas_map'), 'config', 'map_provider.yaml')
    return LaunchDescription([
        Node(
            package='adas_map',
            executable='map_provider',
            name='adas_map_provider',
            output='screen',
            parameters=[config],
        ),
    ])
