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
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from adas_msgs.msg import SafetyStatus


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('adas_launch'), 'launch', 'carla.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestCarlaLaunchSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_carla_launch_smoke')

    def tearDown(self):
        self.node.destroy_node()

    def lifecycle_state(self, name, timeout=5.0):
        client = self.node.create_client(GetState, f'/{name}/get_state')
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result().current_state.id if future.result() else None

    def change_state(self, name, transition_id, timeout=5.0):
        client = self.node.create_client(ChangeState, f'/{name}/change_state')
        self.assertTrue(client.wait_for_service(timeout_sec=timeout))
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result())
        self.assertTrue(future.result().success)

    def test_clean_install_carla_launch_starts_all_nodes(self):
        lifecycle_nodes = {
            'vehicle_interface', 'command_gate', 'safety_monitor', 'aeb',
            'trajectory_follower', 'trajectory_planner', 'behavior_planner',
            'object_tracker',
        }
        expected_nodes = lifecycle_nodes | {'global_planner'}
        deadline = time.monotonic() + 25.0
        observed = set()
        while time.monotonic() < deadline:
            observed = {name.lstrip('/') for name in self.node.get_node_names()}
            if expected_nodes.issubset(observed):
                break
            time.sleep(0.1)
        self.assertTrue(expected_nodes.issubset(observed),
                        f'missing nodes: {sorted(expected_nodes - observed)}')

        for name in lifecycle_nodes:
            state = None
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                state = self.lifecycle_state(name)
                if state == State.PRIMARY_STATE_ACTIVE:
                    break
                time.sleep(0.05)
            self.assertEqual(state, State.PRIMARY_STATE_ACTIVE,
                             f'{name} did not reach active')

        # 必需安全组件停更时必须 fail-closed，不能因诊断过期而恢复为健康。
        self.change_state('aeb', Transition.TRANSITION_DEACTIVATE)
        matched = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.node.create_subscription(
            SafetyStatus, '/adas/system/safety_status',
            lambda msg: matched.append(msg)
            if msg.overall >= SafetyStatus.LEVEL_MRM_COMFORT and
            any('aeb' in item for item in msg.failed_components) else None,
            qos)
        deadline = time.monotonic() + 8.0
        while not matched and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        self.assertTrue(matched, 'AEB diagnostic loss did not request MRM')
