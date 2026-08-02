import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from adas_msgs.msg import AebStatus


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('adas_launch'), 'launch', 'sil_aeb.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestAebScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_aeb_scenario')
        self.qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def tearDown(self):
        self.node.destroy_node()

    def wait_state(self, predicate, timeout):
        matched = []
        sub = self.node.create_subscription(
            AebStatus, '/adas/control/aeb/status',
            lambda msg: matched.append(msg) if predicate(msg) else None, self.qos)
        deadline = time.monotonic() + timeout
        while not matched and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        self.assertTrue(matched, 'timed out waiting for AEB state')
        return matched[-1]

    def test_trigger_and_release(self):
        # 名义触发时刻 ≈ 激活链+起步加速 ~25s；WSL 负载抖动下 35s 窗贴线（实测偶发超时）
        emergency = self.wait_state(
            lambda msg: msg.state == AebStatus.STATE_EMERGENCY, 50.0)
        self.assertLess(emergency.ttc_s, 2.5)
        released = self.wait_state(
            lambda msg: msg.state != AebStatus.STATE_EMERGENCY, 25.0)
        self.assertIn(released.state, [AebStatus.STATE_INACTIVE, AebStatus.STATE_MONITORING])
