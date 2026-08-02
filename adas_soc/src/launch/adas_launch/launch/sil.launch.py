# SIL 全栈闭环 launch（基准场景：直道+弯道，无目标）
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from launch import LaunchDescription  # noqa: E402


def generate_launch_description():
    return LaunchDescription(adas_nodes())
