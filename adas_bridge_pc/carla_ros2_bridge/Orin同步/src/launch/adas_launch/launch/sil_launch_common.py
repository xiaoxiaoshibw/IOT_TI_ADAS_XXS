"""SIL 全栈公共 launch：Lifecycle 状态确认驱动的安全启动链。"""

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import Shutdown, matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _config(name):
    return os.path.join(get_package_share_directory('adas_launch'), 'config', name)


def _change_state(node, transition_id):
    return EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=transition_id,
    ))


def _lifecycle_node(package, executable, name, scenario_files):
    params = [_config(name + '.yaml')]
    for extra in scenario_files:
        params.append(_config(extra))
    return LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace='',
        output='screen',
        parameters=params,
    )


def adas_nodes(sim_extra_params=None, scenario_overlay=None, include_sim=True,
               include_navigation=False, include_mcu=False):
    """返回节点与事件处理器；每个节点 Active 后才配置下一个节点。

    sim_extra_params：场景 overlay yaml 文件名列表——追加到**所有节点**的参数表
    （每个节点只读取自己的 section），场景文件因此可以覆盖任意节点参数
    （如 acc/aeb 场景关闭 behavior 的超车）。
    scenario_overlay：仅传给 sim_vehicle 的 ROS 原生扁平参数字典；用于
    schema-v1 JSON，避免让 C++ SoC 节点引入 JSON 解析依赖。
    include_sim：False = 不起 sim_vehicle（CARLA/HIL 模式——感知话题与执行
    回路由外部桥提供，如 IOT_TI carla_ros2_bridge）。
    include_navigation：启动地图全局规划节点；CARLA 点到点导航模式启用。
    include_mcu：启动 vcan0 网关与 MCU host runner，执行 MCU-in-the-loop SIL。
    """
    scenario_files = list(sim_extra_params or [])
    lifecycle_names = [
        'vehicle_interface', 'command_gate', 'safety_monitor', 'aeb',
        'trajectory_follower', 'trajectory_planner', 'behavior_planner',
        'object_tracker',
    ]
    lifecycle_nodes = [
        _lifecycle_node('adas_vehicle_interface', 'vehicle_interface_node',
                        'vehicle_interface', scenario_files),
        _lifecycle_node('adas_command_gate', 'command_gate_node', 'command_gate',
                        scenario_files),
        _lifecycle_node('adas_safety_monitor', 'safety_monitor_node', 'safety_monitor',
                        scenario_files),
        _lifecycle_node('adas_aeb', 'aeb_node', 'aeb', scenario_files),
        _lifecycle_node('adas_trajectory_follower', 'trajectory_follower_node',
                        'trajectory_follower', scenario_files),
        _lifecycle_node('adas_trajectory_planner', 'trajectory_planner_node',
                        'trajectory_planner', scenario_files),
        _lifecycle_node('adas_behavior_planner', 'behavior_planner_node',
                        'behavior_planner', scenario_files),
        _lifecycle_node('adas_object_tracker', 'object_tracker_node', 'object_tracker',
                        scenario_files),
    ]

    actions = list(lifecycle_nodes)
    if include_navigation:
        actions.append(Node(
            package='adas_global_planner',
            executable='global_planner_node',
            name='global_planner',
            output='screen',
            parameters=[_config('global_planner.yaml')],
        ))
    if include_sim:
        sim_params = [_config('sim_vehicle.yaml')]
        for extra in scenario_files:
            sim_params.append(_config(extra))
        if scenario_overlay:
            sim_params.append(dict(scenario_overlay))
        actions.append(Node(
            package='adas_sim_vehicle',
            executable='sim_vehicle_node',
            name='sim_vehicle',
            output='screen',
            parameters=sim_params,
        ))
    if include_mcu:
        actions.append(Node(
            package='adas_can_gateway',
            executable='can_gateway_node',
            name='can_gateway',
            output='screen',
            parameters=[_config('can_sim.yaml')],
        ))
        runner = os.path.join(
            get_package_prefix('adas_launch'), 'lib', 'adas_launch',
            'mcu_sil_runner.py')
        actions.append(ExecuteProcess(
            cmd=[sys.executable, runner, '--interface', 'vcan0'],
            name='mcu_sil_runner',
            output='screen',
        ))
    for index, node in enumerate(lifecycle_nodes):
        name = lifecycle_names[index]
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg=f'[lifecycle] {name} configured'),
                _change_state(node, Transition.TRANSITION_ACTIVATE),
            ],
        )))

        next_action = (
            _change_state(lifecycle_nodes[index + 1], Transition.TRANSITION_CONFIGURE)
            if index + 1 < len(lifecycle_nodes)
            else LogInfo(msg='[lifecycle] ADAS SoC stack is active')
        )
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='activating',
            goal_state='active',
            entities=[LogInfo(msg=f'[lifecycle] {name} active'), next_action],
        )))

        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='unconfigured',
            entities=[
                LogInfo(msg=f'[lifecycle] ERROR: {name} configure failed; shutting down'),
                EmitEvent(event=Shutdown(reason=f'{name} configure failed')),
            ],
        )))
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='activating',
            goal_state='inactive',
            entities=[
                LogInfo(msg=f'[lifecycle] ERROR: {name} activate failed; shutting down'),
                EmitEvent(event=Shutdown(reason=f'{name} activate failed')),
            ],
        )))

    actions.append(RegisterEventHandler(OnProcessStart(
        target_action=lifecycle_nodes[0],
        on_start=[
            LogInfo(msg='[lifecycle] starting deterministic activation chain'),
            _change_state(lifecycle_nodes[0], Transition.TRANSITION_CONFIGURE),
        ],
    )))
    return actions
