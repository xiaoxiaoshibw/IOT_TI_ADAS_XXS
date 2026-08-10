# CARLA/HIL 全栈 launch（M6）：不起 sim_vehicle，感知话题由外部 CARLA 桥提供。
#
# 部署形态（IOT_TI 架构）：
#   PC（Ubuntu 24.04 + Jazzy）：CARLA + IOT_TI/carla_ros2_bridge 发布
#       /adas/localization/kinematic_state  /adas/perception/lane_state
#       /adas/vehicle/steering_report       /adas/perception/objects_raw
#   Orin Nano（本机，Humble）：ros2 launch adas_launch carla.launch.py
#       栈的执行输出 /adas/vehicle/actuation_cmd 由桥订阅（调通模式），
#       正式闭环改经 Jetson Orin Nano PEAK PCAN-USB (SocketCAN can1) → F280025C → MCU CAN2
#       （调试备源：USB CANalyst-II → MCU CAN2）。
#
# 两端 ROS_DOMAIN_ID 必须一致。
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction  # noqa: E402
from launch.substitutions import LaunchConfiguration  # noqa: E402


def scenario_parameter_files(scenario_id):
    """Return the SoC behavior overlay for a catalog scenario.

    CARLA owns the actors, but the control stack still has to receive the same
    scenario semantics.  In particular ACC/AEB must disable overtaking while
    overtake scenarios keep it enabled.
    """
    if scenario_id in {'acc', 'acc_stop_and_go', 'acc_slow_truck'}:
        return ['acc_scenario.yaml']
    if scenario_id in {'aeb', 'aeb_stationary', 'aeb_pedestrian'}:
        return ['aeb_scenario.yaml']
    if scenario_id in {'overtake', 'dense_overtake_v1'}:
        return ['overtake_scenario.yaml']
    return []


def _setup(context):
    scenario_id = LaunchConfiguration('scenario_id').perform(context).strip()
    overlays = scenario_parameter_files(scenario_id)
    overlay_text = ','.join(overlays) if overlays else 'baseline'
    return [
        LogInfo(msg='[scenario] CARLA SoC profile: %s -> %s' %
                (scenario_id, overlay_text)),
        *adas_nodes(sim_extra_params=overlays, include_sim=False,
                    include_navigation=True),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_id', default_value='free',
            description='Catalog scenario whose behavior policy is applied'),
        OpaqueFunction(function=_setup),
    ])
