# SIL A/B 横向控制器对比（Commit 7）
#
# 目的：让 PurePursuit（默认）与 LQR 同时订阅同一组规划/感知话题，并各自
# 把控制命令发到不同 topic，便于用 run_scenario_metrics.py 分别抓 metrics
# 对比。**不在此 launch 中切换默认**——默认仍是 pure_pursuit（见
# trajectory_follower.yaml:7）。LQR 节点以 "lqr" 命名空间启动，输出 topic
# remap 为 /lqr/adas/control/trajectory_follower/control_cmd。
#
# 验收门槛（不切默认）：LQR 在某场景的 P95 lat / max steer rate / P95 jerk
# 三项中任一显著优于 PP 才在独立 follow-up commit 把 trajectory_follower.yaml
# 的 lateral_controller_mode 改成 lqr。
#
# 主栈 follower 与 LQR 影子节点同名（trajectory_follower）但不同命名空间：
# 必须用 matches_action(node) 精确锁定 Lifecycle 事件目标，避免 ChangeState
# 按名字命中主栈节点（那样 LQR 影子永远不会被 configure）。

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes, _config  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler  # noqa: E402
from launch.event_handlers import OnProcessStart  # noqa: E402
from launch.events import Shutdown, matches_action  # noqa: E402
from launch_ros.actions import LifecycleNode  # noqa: E402
from launch_ros.event_handlers import OnStateTransition  # noqa: E402
from launch_ros.events.lifecycle import ChangeState  # noqa: E402
from lifecycle_msgs.msg import Transition  # noqa: E402


def _change_state(node, transition_id):
    return EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=transition_id,
    ))


def generate_launch_description():
    # 主栈用默认 Pure Pursuit 横向控制器（sil_launch_common.adas_nodes 内部
    # 已经按 trajectory_follower.yaml 的 lateral_controller_mode 选择）
    main_stack = adas_nodes(sim_extra_params=[], include_navigation=True)

    # LQR 影子节点：相同包/可执行，但 namespace=/lqr + 参数 overlay
    # + 输出 topic remap。它和主栈的 trajectory_follower_node 平行存在，
    # 各跑各的，offline 用 run_scenario_metrics.py 分别抓 /pp/ 和 /lqr/ 输出。
    lqr_follower_node = LifecycleNode(
        package='adas_trajectory_follower',
        executable='trajectory_follower_node',
        name='trajectory_follower',
        namespace='lqr',
        output='screen',
        parameters=[
            _config('trajectory_follower.yaml'),
            _config('lqr_lateral.yaml'),
        ],
        remappings=[
            ('/adas/control/trajectory_follower/control_cmd',
             '/lqr/adas/control/trajectory_follower/control_cmd'),
        ],
    )

    # LQR follower 独立于主栈启动：进程起来后 configure，configure 完成
    # 即 activate（影子节点，不需要等待主栈生命周期）。
    lqr_actions = [lqr_follower_node]
    lqr_actions.append(RegisterEventHandler(OnProcessStart(
        target_action=lqr_follower_node,
        on_start=[
            LogInfo(msg='[lifecycle] lqr follower: starting deterministic activation'),
            _change_state(lqr_follower_node, Transition.TRANSITION_CONFIGURE),
        ],
    )))
    lqr_actions.append(RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lqr_follower_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[
            LogInfo(msg='[lifecycle] lqr follower configured'),
            _change_state(lqr_follower_node, Transition.TRANSITION_ACTIVATE),
        ],
    )))
    lqr_actions.append(RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lqr_follower_node,
        start_state='activating',
        goal_state='active',
        entities=[LogInfo(msg='[lifecycle] lqr follower active (A/B 影子在线)')],
    )))
    # 配置/激活失败即整体退出——和主栈一致：fail-closed。
    lqr_actions.append(RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lqr_follower_node,
        start_state='configuring',
        goal_state='unconfigured',
        entities=[
            LogInfo(msg='[lifecycle] ERROR: lqr follower configure failed; shutting down'),
            EmitEvent(event=Shutdown(reason='lqr follower configure failed')),
        ],
    )))
    lqr_actions.append(RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lqr_follower_node,
        start_state='activating',
        goal_state='inactive',
        entities=[
            LogInfo(msg='[lifecycle] ERROR: lqr follower activate failed; shutting down'),
            EmitEvent(event=Shutdown(reason='lqr follower activate failed')),
        ],
    )))

    return LaunchDescription(main_stack + lqr_actions)
