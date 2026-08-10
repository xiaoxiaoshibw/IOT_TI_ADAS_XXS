"""MIL: production SoC stack + software vehicle + vcan + MCU host runner."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402
from scenario_overlay import load_scenario_overlay  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # noqa: E402
from launch.substitutions import LaunchConfiguration  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


def _config(name):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('adas_launch'), 'config', name)


def _actions(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    scenario_file = LaunchConfiguration('scenario_file').perform(context)
    scenario_id = LaunchConfiguration('scenario_id').perform(context)
    seed_text = LaunchConfiguration('seed').perform(context)
    extras = ([scenario] if scenario and not (scenario_file or scenario_id)
              else [])
    overlay = None
    if scenario_file or scenario_id:
        seed = int(seed_text) if seed_text else None
        metadata, overlay = load_scenario_overlay(
            scenario_file=scenario_file, scenario_id=scenario_id, seed=seed)
        print('[scenario] schema=1 id=%s seed=%d actors=%d source=%s' % (
            metadata['id'], metadata['seed'], metadata['actor_count'],
            metadata['source_file']))
    actions = adas_nodes(
        sim_extra_params=extras, scenario_overlay=overlay,
        include_sim=True, include_navigation=True)
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
        DeclareLaunchArgument(
            'scenario_file', default_value='',
            description='schema-v1 JSON scenario; takes priority over legacy yaml'),
        DeclareLaunchArgument(
            'scenario_id', default_value='',
            description='expected stable scenario ID or catalog lookup ID'),
        DeclareLaunchArgument(
            'seed', default_value='',
            description='optional unsigned deterministic seed override'),
        OpaqueFunction(function=_actions),
    ])
