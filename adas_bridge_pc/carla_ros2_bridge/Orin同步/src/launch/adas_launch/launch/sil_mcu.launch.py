"""PC SIL with the production CAN gateway and MCU safety core on vcan0."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # noqa: E402
from launch.substitutions import LaunchConfiguration  # noqa: E402


def _build_actions(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    extras = [scenario] if scenario else []
    return adas_nodes(sim_extra_params=extras, include_mcu=True)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='optional adas_launch scenario overlay yaml'),
        OpaqueFunction(function=_build_actions),
    ])
