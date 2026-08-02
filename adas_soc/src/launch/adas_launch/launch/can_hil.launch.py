"""CARLA/DDS → SoC → SocketCAN → MCU 的正式 HIL 启动入口。"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch import LaunchDescription  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit  # noqa: E402
from launch.events import Shutdown  # noqa: E402
from launch.substitutions import LaunchConfiguration  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('adas_launch'), 'config', 'can_hil.yaml')
    config = LaunchConfiguration('params_file')
    actions = adas_nodes(
        sim_extra_params=['can_hil.yaml'], include_sim=False, include_navigation=True)
    actions.insert(0, DeclareLaunchArgument(
        'params_file', default_value=default_config,
        description='Host-specific CAN HIL parameter file'))
    gateway = Node(
        package='adas_can_gateway', executable='can_gateway_node', name='can_gateway',
        output='screen', parameters=[config])
    actions.append(gateway)
    actions.append(Node(
        package='adas_dtc_recorder', executable='dtc_recorder', name='dtc_recorder',
        output='screen', parameters=[{
            'history_path': '/var/log/adas/dtc_history.json',
            'max_records': 128,
        }]))
    actions.append(Node(
        package='adas_resource_monitor', executable='resource_monitor',
        name='resource_monitor', output='screen', parameters=[{
            'can_interface': 'can1',
            'disk_path': '/var/log/adas',
            'log_path': '/var/log/adas',
            'rate_hz': 1.0,
        }, config]))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=gateway,
        on_exit=[EmitEvent(event=Shutdown(reason='CAN gateway exited'))],
    )))
    return LaunchDescription(actions)
