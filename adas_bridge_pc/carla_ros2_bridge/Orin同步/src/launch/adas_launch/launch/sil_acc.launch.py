# SIL ACC 跟车场景（M2）：sim_vehicle 叠加 acc_scenario.yaml
# （1000m 直道 + 脚本前车：60m 处 10m/s，t=35s 刹停，t=55s 重新起步）
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from launch import LaunchDescription  # noqa: E402


def generate_launch_description():
    return LaunchDescription(adas_nodes(sim_extra_params=['acc_scenario.yaml']))
