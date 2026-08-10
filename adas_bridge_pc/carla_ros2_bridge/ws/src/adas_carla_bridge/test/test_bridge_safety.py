import math
import json
from types import SimpleNamespace
import uuid

import rclpy
from adas_msgs.msg import ActuationCommand
from std_msgs.msg import String

from adas_carla_bridge.bridge_node import CarlaBridgeNode, validate_actuation_values


def test_valid_actuation_is_accepted():
    assert validate_actuation_values(0.5, 0.0, -0.25) == (True, 'ok')
    assert validate_actuation_values(0.0, 1.0, 1.0) == (True, 'ok')


def test_non_finite_actuation_is_rejected():
    for value in (math.nan, math.inf, -math.inf):
        assert not validate_actuation_values(value, 0.0, 0.0)[0]
        assert not validate_actuation_values(0.0, value, 0.0)[0]
        assert not validate_actuation_values(0.0, 0.0, value)[0]


def test_out_of_range_actuation_is_rejected():
    assert not validate_actuation_values(-0.01, 0.0, 0.0)[0]
    assert not validate_actuation_values(1.01, 0.0, 0.0)[0]
    assert not validate_actuation_values(0.0, -0.01, 0.0)[0]
    assert not validate_actuation_values(0.0, 1.01, 0.0)[0]
    assert not validate_actuation_values(0.0, 0.0, -1.01)[0]
    assert not validate_actuation_values(0.0, 0.0, 1.01)[0]


def test_throttle_brake_conflict_is_rejected():
    valid, reason = validate_actuation_values(0.2, 0.3, 0.0)
    assert not valid
    assert reason == 'throttle_brake_conflict'


def test_invalid_frame_latches_failsafe_until_three_valid_frames():
    rclpy.init()
    node = CarlaBridgeNode(SimpleNamespace(
        stale_timeout_s=0.5, control_source='ros2', scenario='test'))
    try:
        invalid = ActuationCommand()
        invalid.throttle = math.nan
        node._actuation_cb(invalid)
        assert node.get_actuation()['invalid_latched']
        assert node.get_actuation()['brake'] == 1.0

        valid = ActuationCommand()
        valid.throttle = 0.2
        valid.brake = 0.0
        valid.steer = 0.1
        for _ in range(2):
            node._actuation_cb(valid)
            assert node.get_actuation()['invalid_latched']
        node._actuation_cb(valid)
        recovered = node.get_actuation()
        assert not recovered['invalid_latched']
        assert recovered['throttle'] == 0.2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fault_ack_requires_uuid4_and_reports_can_contract():
    class Receiver:
        def send_fault_injection(self, command, parameter, sequence):
            assert (command, parameter, sequence) == (3, 9, 1)
            return {'system_state': 4, 'fault_level': 1, 'sequence': sequence}

        def close(self):
            pass

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(json.loads(message.data))

    rclpy.init()
    node = CarlaBridgeNode(SimpleNamespace(
        stale_timeout_s=0.5, control_source='ros2', scenario='test'))
    publisher = Publisher()
    node.pub_fault_ack = publisher
    node._can_receiver = Receiver()
    try:
        invalid = String()
        invalid.data = json.dumps({'request_id': 'not-a-uuid', 'cmd': 3, 'param': 9})
        node._fault_inject_cb(invalid)
        assert not publisher.messages[-1]['accepted']

        request_id = str(uuid.uuid4())
        valid = String()
        valid.data = json.dumps({'request_id': request_id, 'cmd': 3, 'param': 9})
        node._fault_inject_cb(valid)
        ack = publisher.messages[-1]
        assert ack['accepted']
        assert ack['request_id'] == request_id
        assert ack['can_id'] == '0x302'
        assert ack['dlc'] == 8
        assert ack['crc_valid']
        assert ack['param'] == 9
    finally:
        node.destroy_node()
        rclpy.shutdown()
