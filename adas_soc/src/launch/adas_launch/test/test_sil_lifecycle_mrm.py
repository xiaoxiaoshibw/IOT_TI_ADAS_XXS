import math
import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from adas_msgs.msg import ActuationCommand, GateStatus, LaneState, SafetyStatus


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('adas_launch'), 'launch', 'sil.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestLifecycleMrm(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_lifecycle_mrm')
        self.latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def tearDown(self):
        self.node.destroy_node()

    def wait_message(self, msg_type, topic, predicate, timeout, qos=10):
        matched = []
        sub = self.node.create_subscription(
            msg_type, topic, lambda msg: matched.append(msg) if predicate(msg) else None, qos)
        deadline = time.monotonic() + timeout
        while not matched and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        self.assertTrue(matched, f'timed out waiting for {topic}')
        return matched[-1]

    def lifecycle_state(self, name, timeout=10.0):
        client = self.node.create_client(GetState, f'/{name}/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), f'{name} get_state unavailable')
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result(), f'{name} get_state failed')
        return future.result().current_state.id

    def change_state(self, name, transition_id, timeout=10.0):
        client = self.node.create_client(ChangeState, f'/{name}/change_state')
        self.assertTrue(client.wait_for_service(timeout_sec=timeout))
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result())
        self.assertTrue(future.result().success, f'{name} transition {transition_id} failed')

    def test_full_lifecycle_lka_and_fail_safe(self):
        lifecycle_nodes = [
            'vehicle_interface', 'command_gate', 'safety_monitor', 'aeb',
            'trajectory_follower', 'trajectory_planner', 'behavior_planner',
            'object_tracker',
        ]
        for name in lifecycle_nodes:
            deadline = time.monotonic() + 20.0
            state = None
            while time.monotonic() < deadline:
                state = self.lifecycle_state(name)
                if state == State.PRIMARY_STATE_ACTIVE:
                    break
                time.sleep(0.05)
            self.assertEqual(state, State.PRIMARY_STATE_ACTIVE, f'{name} not active')

        seen_diagnostics = set()

        def diagnostics_ready(msg):
            for status in msg.status:
                for name in lifecycle_nodes:
                    if name in status.name:
                        seen_diagnostics.add(name)
            return len(seen_diagnostics) == len(lifecycle_nodes)

        # 诊断 1Hz/节点，激活链完成后首帧需数秒；WSL 负载抖动下 10s 贴线（实测偶发超时）
        self.wait_message(DiagnosticArray, '/diagnostics', diagnostics_ready, 30.0, 20)

        self.wait_message(
            GateStatus, '/adas/control/gate/status',
            lambda msg: msg.selected_source == GateStatus.SOURCE_FOLLOWER,
            25.0, self.latched_qos)
        self.wait_message(
            Odometry, '/adas/localization/kinematic_state',
            lambda msg: msg.twist.twist.linear.x >= 14.0,
            25.0, qos_profile_sensor_data)

        offsets = []
        sub = self.node.create_subscription(
            LaneState, '/adas/perception/lane_state',
            lambda msg: offsets.append(msg.lateral_offset), qos_profile_sensor_data)
        deadline = time.monotonic() + 8.0
        while len(offsets) < 50 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        self.assertGreaterEqual(len(offsets), 20)
        rms = math.sqrt(sum(value * value for value in offsets) / len(offsets))
        self.assertLessEqual(rms, 0.3, f'LKA lateral RMS regressed: {rms:.3f} m')

        self.change_state('object_tracker', Transition.TRANSITION_DEACTIVATE)
        self.wait_message(
            SafetyStatus, '/adas/system/safety_status',
            lambda msg: msg.overall >= SafetyStatus.LEVEL_MRM_COMFORT,
            8.0, self.latched_qos)
        self.wait_message(
            GateStatus, '/adas/control/gate/status',
            lambda msg: msg.selected_source == GateStatus.SOURCE_BUILTIN_STOP,
            5.0, self.latched_qos)

        self.change_state('command_gate', Transition.TRANSITION_DEACTIVATE)
        self.wait_message(
            ActuationCommand, '/adas/vehicle/actuation_cmd',
            lambda msg: msg.brake >= 0.99 and msg.throttle <= 0.01,
            3.0, 10)
