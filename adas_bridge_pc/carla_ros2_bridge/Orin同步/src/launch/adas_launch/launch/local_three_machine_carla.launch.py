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


def _scenario_parameter_files(scenario_id):
    if scenario_id in {'acc', 'acc_stop_and_go', 'acc_slow_truck'}:
        return ['acc_scenario.yaml']
    if scenario_id in {'aeb', 'aeb_stationary', 'aeb_pedestrian'}:
        return ['aeb_scenario.yaml']
    if scenario_id in {'overtake', 'dense_overtake_v1'}:
        return ['overtake_scenario.yaml']
    return []


def _actions(context):
    run_id = LaunchConfiguration('run_id').perform(context).strip()
    scenario_id = LaunchConfiguration('scenario_id').perform(context).strip()
    actions = adas_nodes(
        sim_extra_params=_scenario_parameter_files(scenario_id),
        include_sim=False, include_navigation=True, run_id=run_id)
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
        DeclareLaunchArgument('scenario_id', default_value='free',
                              description='Catalog scenario behavior profile'),
        OpaqueFunction(function=_actions),
    ])
