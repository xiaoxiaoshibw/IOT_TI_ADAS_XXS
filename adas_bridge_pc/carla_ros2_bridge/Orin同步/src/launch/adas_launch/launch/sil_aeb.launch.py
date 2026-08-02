# SIL AEB 横穿行人场景（M3）：sim_vehicle 叠加 aeb_scenario.yaml
# （500m 直道，行人在 150m 处、自车逼近 35m 时以 1.5m/s 横穿）
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from launch import LaunchDescription  # noqa: E402


def generate_launch_description():
    return LaunchDescription(adas_nodes(sim_extra_params=['aeb_scenario.yaml']))
