# SIL 双栈冗余场景（M5）：
#   全局：sim_vehicle（世界）+ vehicle_interface（唯一执行器）+ redundancy_arbiter（仲裁）
#   双栈：/primary /backup 各 7 节点（gate/aeb/safety_monitor/follower/planner/behavior/tracker），
#         栈内 topic 重映射隔离；世界真值 topic（odom/lane/steering/objects_raw）全局共享。
#   仲裁器订阅两栈 gate 输出 → 发布全局 /adas/control/gate/control_cmd 给 vehicle_interface。
# 激活链：vehicle_interface → arbiter → primary 栈 7 节点 → backup 栈 7 节点（确定性顺序）。
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import Shutdown, matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# 栈内（须按角色隔离的）topic
STACK_TOPICS = [
    '/adas/perception/objects',
    '/adas/planning/behavior',
    '/adas/planning/trajectory',
    '/adas/control/trajectory_follower/control_cmd',
    '/adas/control/aeb/status',
    '/adas/control/aeb/emergency_cmd',
    '/adas/control/gate/control_cmd',
    '/adas/control/gate/status',
    '/adas/system/safety_status',
    # /diagnostics 必须隔离：否则备栈 safety_monitor 会收到主栈死亡节点的
    # ERROR 诊断，误判自家组件故障 → MRM 振荡拖停（M5 实测教训）
    '/diagnostics',
]

STACK_NODES = [
    ('adas_command_gate', 'command_gate_node', 'command_gate'),
    ('adas_aeb', 'aeb_node', 'aeb'),
    ('adas_safety_monitor', 'safety_monitor_node', 'safety_monitor'),
    ('adas_trajectory_follower', 'trajectory_follower_node', 'trajectory_follower'),
    ('adas_trajectory_planner', 'trajectory_planner_node', 'trajectory_planner'),
    ('adas_behavior_planner', 'behavior_planner_node', 'behavior_planner'),
    ('adas_object_tracker', 'object_tracker_node', 'object_tracker'),
]


def _config(name):
    return os.path.join(get_package_share_directory('adas_launch'), 'config', name)


def _change_state(node, transition_id):
    return EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=transition_id,
    ))


def _stack_node(package, executable, name, role):
    remaps = [(t, f'/{role}{t}') for t in STACK_TOPICS]
    parameters = [_config(name + '.yaml')]
    if name == 'safety_monitor':
        # /diagnostics is intentionally isolated per stack. The global
        # vehicle_interface diagnostic therefore belongs to the arbiter/global
        # execution path, not to either namespaced stack monitor.
        parameters.append({
            'required_diagnostic_components': [
                'command_gate', 'trajectory_follower', 'trajectory_planner',
                'behavior_planner', 'object_tracker', 'aeb',
            ],
        })
    return LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace=role,
        output='screen',
        parameters=parameters,
        remappings=remaps,
    )


def generate_launch_description():
    # 全局节点
    vehicle_interface = LifecycleNode(
        package='adas_vehicle_interface', executable='vehicle_interface_node',
        name='vehicle_interface', namespace='', output='screen',
        parameters=[_config('vehicle_interface.yaml')])
    arbiter = LifecycleNode(
        package='adas_redundancy', executable='redundancy_arbiter_node',
        name='redundancy_arbiter', namespace='', output='screen',
        parameters=[_config('redundancy_arbiter.yaml')])
    sim_node = Node(
        package='adas_sim_vehicle', executable='sim_vehicle_node', name='sim_vehicle',
        output='screen', parameters=[_config('sim_vehicle.yaml')])

    ordered = [('vehicle_interface', vehicle_interface), ('redundancy_arbiter', arbiter)]
    for role in ('primary', 'backup'):
        for package, executable, name in STACK_NODES:
            ordered.append((f'{role}/{name}',
                            _stack_node(package, executable, name, role)))

    actions = [node for _, node in ordered] + [sim_node]
    for index, (label, node) in enumerate(ordered):
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring', goal_state='inactive',
            entities=[LogInfo(msg=f'[lifecycle] {label} configured'),
                      _change_state(node, Transition.TRANSITION_ACTIVATE)],
        )))
        next_action = (
            _change_state(ordered[index + 1][1], Transition.TRANSITION_CONFIGURE)
            if index + 1 < len(ordered)
            else LogInfo(msg='[lifecycle] redundant ADAS stacks are active')
        )
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='activating', goal_state='active',
            entities=[LogInfo(msg=f'[lifecycle] {label} active'), next_action],
        )))
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring', goal_state='unconfigured',
            entities=[LogInfo(msg=f'[lifecycle] ERROR: {label} configure failed'),
                      EmitEvent(event=Shutdown(reason=f'{label} configure failed'))],
        )))

    actions.append(RegisterEventHandler(OnProcessStart(
        target_action=vehicle_interface,
        on_start=[LogInfo(msg='[lifecycle] starting redundant activation chain'),
                  _change_state(vehicle_interface, Transition.TRANSITION_CONFIGURE)],
    )))
    return LaunchDescription(actions)
