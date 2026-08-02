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
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from adas_msgs.msg import TrackedObjectArray


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('adas_launch'), 'launch', 'sil_acc.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestAccRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_acc_regression')
        self.speed = 0.0
        self.gap = -1.0
        self.node.create_subscription(
            Odometry, '/adas/localization/kinematic_state', self.on_odom,
            qos_profile_sensor_data)
        self.node.create_subscription(
            TrackedObjectArray, '/adas/perception/objects', self.on_objects,
            qos_profile_sensor_data)

    def tearDown(self):
        self.node.destroy_node()

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x

    def on_objects(self, msg):
        if msg.primary_lead_id >= 0:
            self.gap = msg.primary_lead_gap_m

    def wait_condition(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(predicate(), f'timed out waiting for {description}: speed={self.speed:.2f}, gap={self.gap:.2f}')

    def test_follow_stop_and_restart(self):
        self.wait_condition(
            lambda: 9.0 <= self.speed <= 11.0 and self.gap > 0.0 and
            1.3 <= self.gap / self.speed <= 1.7,
            35.0, 'steady following')
        time_gap = self.gap / max(self.speed, 0.1)
        self.assertGreaterEqual(time_gap, 1.3)
        self.assertLessEqual(time_gap, 1.7)

        self.wait_condition(
            lambda: self.speed <= 0.2 and self.gap >= 4.0,
            25.0, 'standstill behind lead vehicle')
        stopped_gap = self.gap
        self.assertGreaterEqual(stopped_gap, 4.0)

        self.wait_condition(lambda: self.speed >= 8.0, 30.0, 'restart following')
