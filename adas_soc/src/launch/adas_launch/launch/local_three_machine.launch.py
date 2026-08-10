"""Local PC/Orin/MCU HIL: production stack + software vehicle + vcan gateway."""

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
    scenario = LaunchConfiguration('scenario').perform(context)
    extras = [scenario] if scenario else []
    actions = adas_nodes(
        sim_extra_params=extras, include_sim=True, include_navigation=True)
    actions.append(Node(
        package='adas_can_gateway', executable='can_gateway_node',
        name='can_gateway', output='screen',
        parameters=[_config('can_sim.yaml')],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='optional scenario overlay yaml'),
        OpaqueFunction(function=_actions),
    ])
