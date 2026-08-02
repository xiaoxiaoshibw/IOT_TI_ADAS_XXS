import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from launch_ros.actions import LifecycleNode
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState


@pytest.mark.launch_test
def generate_test_description():
    bad_node = LifecycleNode(
        package='adas_vehicle_interface',
        executable='vehicle_interface_node',
        name='bad_vehicle_interface',
        namespace='',
        parameters=[{'rate_hz': 0.0}],
        output='screen',
    )
    return launch.LaunchDescription([bad_node, launch_testing.actions.ReadyToTest()])


class TestInvalidParameters(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_configure_rejected(self):
        node = rclpy.create_node('test_invalid_parameters')
        change = node.create_client(ChangeState, '/bad_vehicle_interface/change_state')
        self.assertTrue(change.wait_for_service(timeout_sec=10.0))
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        future = change.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result())
        self.assertFalse(future.result().success)

        get_state = node.create_client(GetState, '/bad_vehicle_interface/get_state')
        self.assertTrue(get_state.wait_for_service(timeout_sec=5.0))
        state_future = get_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, state_future, timeout_sec=5.0)
        self.assertEqual(state_future.result().current_state.id, State.PRIMARY_STATE_UNCONFIGURED)
        node.destroy_node()
