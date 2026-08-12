"""Local CARLA/Orin/MCU HIL: no software vehicle, vcan0 gateway only."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # noqa: E402
from launch.substitutions import LaunchConfiguration  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


def _config(name):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('adas_launch'), 'config', name)


def _actions(context):
    run_id = LaunchConfiguration('run_id').perform(context).strip()
    actions = adas_nodes(include_sim=False, include_navigation=True, run_id=run_id)
    actions.append(Node(
        package='adas_can_gateway', executable='can_gateway_node',
        name='can_gateway', output='screen',
        parameters=[_config('can_sim.yaml')],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_id', default_value='',
                              description='Expected non-empty run session ID'),
        OpaqueFunction(function=_actions),
    ])
