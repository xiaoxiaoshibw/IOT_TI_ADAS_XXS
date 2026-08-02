# SIL 慢车超越场景（M4）：全节点叠加 overtake_scenario.yaml
# （1200m 直道 + 6m/s 慢车；超车默认开启）
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sil_launch_common import adas_nodes  # noqa: E402

from launch import LaunchDescription  # noqa: E402


def generate_launch_description():
    return LaunchDescription(adas_nodes(sim_extra_params=['overtake_scenario.yaml']))
