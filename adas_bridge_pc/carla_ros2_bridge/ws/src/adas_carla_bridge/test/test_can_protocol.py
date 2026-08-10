from collections import deque
import struct
import time
from types import SimpleNamespace

from adas_carla_bridge.can_protocol import (
    CAN_EFF_FLAG, CAN_FILTER, CAN_RECEIVE_IDS, CAN_RTR_FLAG, CAN_SFF_MASK,
    CanalystReceiver, CANID_FAULT_INJECT, CANID_FAULT_RESPONSE,
    CANID_MCU_CONTROL, CANID_MCU_E2E_DIAG, CANID_MCU_HEARTBEAT, crc8,
    decode_fault_response, encode_fault_injection, frame_crc,
    McuFeedbackGuard, PROTOCOL_VERSION, sequence_forward,
    socketcan_feedback_filters,
)


def finish(can_id, values):
    data = bytearray(values)
    data[7] = frame_crc(can_id, data)
    return bytes(data)


def test_crc_and_sequence_contract():
    assert crc8(b'123456789') == 0xA2
    assert sequence_forward(255, 0)
    assert sequence_forward(1, 2)
    assert not sequence_forward(2, 2)
    assert not sequence_forward(2, 1)


def test_fault_injection_frame_matches_can_v3_contract():
    payload = encode_fault_injection(7, 0x34, 0x56)
    assert payload[:5] == bytes((7, 0x34, 0, 0, 0))
    assert payload[5:7] == bytes((0x56, 0))
    assert payload[7] == frame_crc(CANID_FAULT_INJECT, payload)


def test_fault_injection_rejects_values_outside_wire_width():
    for values in ((-1, 0, 0), (10, 0, 0), (0, -1, 0),
                   (0, 0x100, 0), (0, 0, 0x100)):
        try:
            encode_fault_injection(*values)
            assert False, values
        except ValueError:
            pass


def test_fault_response_requires_crc_and_exposes_mcu_state():
    payload = finish(CANID_FAULT_RESPONSE, [7, 0, 4, 2, 9, 12, 0x40, 0])
    response = decode_fault_response(payload)
    assert response == {'command': 7, 'parameter': 0, 'system_state': 4,
                        'fault_level': 2, 'alive': 9, 'sequence': 12,
                        'fault_code_low': 0x40}
    try:
        decode_fault_response(bytes(8))
        assert False
    except ValueError:
        pass


def test_socketcan_filters_accept_only_feedback_and_fault_ack_ids():
    encoded = socketcan_feedback_filters()
    assert len(encoded) == len(CAN_RECEIVE_IDS) * CAN_FILTER.size
    filters = [CAN_FILTER.unpack_from(encoded, offset)
               for offset in range(0, len(encoded), CAN_FILTER.size)]
    assert [can_id for can_id, _ in filters] == list(CAN_RECEIVE_IDS)
    expected_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG
    assert all(mask == expected_mask for _, mask in filters)


def test_guard_requires_health_and_three_control_frames():
    now = [100.0]
    guard = McuFeedbackGuard(clock=lambda: now[0])
    heartbeat = finish(CANID_MCU_HEARTBEAT, [2, 1, 0, 0, 0, 1, 10, 0])
    e2e = finish(CANID_MCU_E2E_DIAG, [0, 0, 7, 0, 0, 0, PROTOCOL_VERSION, 0])
    assert guard.feed(CANID_MCU_HEARTBEAT, heartbeat)
    assert guard.feed(CANID_MCU_E2E_DIAG, e2e)
    for seq in range(3):
        payload = bytearray(8)
        struct.pack_into('<hh', payload, 0, 300, -1200)
        payload[5] = 75
        payload[6] = seq
        assert guard.feed(CANID_MCU_CONTROL,
                          finish(CANID_MCU_CONTROL, payload))
    current = guard.current()
    assert not current['invalid_latched']
    assert current['brake'] == 0.75
    assert current['steer'] == 0.1


def test_guard_rejects_duplicate_and_times_out_fail_closed():
    now = [10.0]
    guard = McuFeedbackGuard(clock=lambda: now[0])
    guard.feed(CANID_MCU_HEARTBEAT,
               finish(CANID_MCU_HEARTBEAT, [2, 1, 0, 0, 0, 1, 10, 0]))
    guard.feed(CANID_MCU_E2E_DIAG,
               finish(CANID_MCU_E2E_DIAG, [0, 0, 7, 0, 0, 0, PROTOCOL_VERSION, 0]))
    control = finish(CANID_MCU_CONTROL, [0, 0, 0, 0, 0, 0, 1, 0])
    assert guard.feed(CANID_MCU_CONTROL, control)
    assert not guard.feed(CANID_MCU_CONTROL, control)
    assert guard.current()['invalid_latched']
    now[0] += 1.0
    assert guard.current()['brake'] == 1.0


def test_guard_rejects_wrong_crc_and_protocol_version():
    now = [50.0]
    guard = McuFeedbackGuard(clock=lambda: now[0])
    assert not guard.feed(CANID_MCU_CONTROL, bytes(8))
    # 先把守卫喂到健康解锁，再喂错误版本，确认真正触发重新闭锁
    guard.feed(CANID_MCU_HEARTBEAT,
               finish(CANID_MCU_HEARTBEAT, [2, 1, 0, 0, 0, 1, 10, 0]))
    guard.feed(CANID_MCU_E2E_DIAG,
               finish(CANID_MCU_E2E_DIAG, [0, 0, 7, 0, 0, 0, PROTOCOL_VERSION, 0]))
    for seq in range(1, 4):
        payload = bytearray(8)
        payload[6] = seq
        assert guard.feed(CANID_MCU_CONTROL, finish(CANID_MCU_CONTROL, payload))
    assert not guard.current()['invalid_latched']
    bad_version = finish(
        CANID_MCU_E2E_DIAG, [0, 0, 7, 0, 0, 0, PROTOCOL_VERSION - 1, 0])
    assert guard.feed(CANID_MCU_E2E_DIAG, bad_version)
    assert guard.current()['invalid_latched']


class FakeCanalystBus:
    def __init__(self, messages, auto_ack=False, **kwargs):
        self.messages = deque(messages)
        self.auto_ack = auto_ack
        self.kwargs = kwargs
        self.closed = False
        self.sent = []

    def recv(self, timeout):
        if self.messages:
            return self.messages.popleft()
        time.sleep(min(timeout, 0.001))
        return None

    def send(self, message):
        self.sent.append(message)
        if self.auto_ack:
            command, parameter = message.data[:2]
            self.messages.append(SimpleNamespace(
                arbitration_id=CANID_FAULT_RESPONSE,
                data=finish(CANID_FAULT_RESPONSE,
                            [command, parameter, 3, 1, 4, 8, 0, 0]),
                dlc=8, is_extended_id=False, is_remote_frame=False))

    def shutdown(self):
        self.closed = True


def test_canalyst_receiver_configures_backend_and_feeds_guard():
    heartbeat = finish(
        CANID_MCU_HEARTBEAT, [2, 1, 0, 0, 0, 1, 10, 0])
    e2e = finish(
        CANID_MCU_E2E_DIAG,
        [0, 0, 7, 0, 0, 0, PROTOCOL_VERSION, 0])
    messages = []
    for can_id, data in ((CANID_MCU_HEARTBEAT, heartbeat),
                         (CANID_MCU_E2E_DIAG, e2e)):
        messages.append(SimpleNamespace(
            arbitration_id=can_id, data=data, dlc=8,
            is_extended_id=False, is_remote_frame=False))
    for seq in range(3):
        payload = bytearray(8)
        payload[4] = 25
        payload[6] = seq
        messages.append(SimpleNamespace(
            arbitration_id=CANID_MCU_CONTROL,
            data=finish(CANID_MCU_CONTROL, payload), dlc=8,
            is_extended_id=False, is_remote_frame=False))
    captured = {}

    def factory(**kwargs):
        captured.update(kwargs)
        return FakeCanalystBus(messages, **kwargs)

    receiver = CanalystReceiver(
        device=2, channel=1, bitrate=500000, bus_factory=factory)
    try:
        deadline = time.monotonic() + 1.0
        while receiver.guard.valid_count < 3 and time.monotonic() < deadline:
            time.sleep(0.001)
        assert receiver.current()['throttle'] == 0.25
        assert captured['interface'] == 'canalystii'
        assert captured['device'] == 2
        assert captured['channel'] == 1
        assert captured['bitrate'] == 500000
        assert [item['can_id'] for item in captured['can_filters']] == \
            list(CAN_RECEIVE_IDS)
    finally:
        receiver.close()


def test_canalyst_fault_request_waits_for_matching_mcu_ack():
    bus = None

    def factory(**kwargs):
        nonlocal bus
        bus = FakeCanalystBus([], auto_ack=True, **kwargs)
        return bus

    receiver = CanalystReceiver(bus_factory=factory)
    try:
        response = receiver.send_fault_injection(7, 0, 22)
        assert response['command'] == 7
        assert response['system_state'] == 3
        assert response['sequence'] == 8
        assert bus.sent[0].arbitration_id == CANID_FAULT_INJECT
    finally:
        receiver.close()
