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


def generate_launch_description():
    return LaunchDescription(adas_nodes(include_sim=False, include_navigation=True))
