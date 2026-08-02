# SIL 基准场景 + LQR 横向控制器（M7 实验插件在线验证；基准缓弯为其适用工况）
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from launch import LaunchDescription  # noqa: E402


def generate_launch_description():
    return LaunchDescription(adas_nodes(sim_extra_params=['lqr_lateral.yaml']))
