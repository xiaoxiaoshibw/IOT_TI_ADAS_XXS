"""MCU CAN v3 feedback authentication and CAN receive backends.

v3 的 CRC8、序列号前向校验、心跳/E2E 健康监控、字段编码与 v2 字节级兼容；
会话状态机（0x104/0x206）2026-07 重构后由 MCU 自驱授权（自动 READY→ARMED→
ACTIVE），SoC 侧 hil_session_manager 只观测 0x206 推进要发的 0x104 phase，
无需物理按钮/TCP ARM。本模块只负责反馈帧链路层认证。
"""

import math
import socket
import struct
import threading
import time

CANID_MCU_CONTROL = 0x201
CANID_MCU_HEARTBEAT = 0x202
CANID_MCU_DIAG = 0x203
CANID_MCU_E2E_DIAG = 0x204
CANID_FAULT_INJECT = 0x301
CANID_FAULT_RESPONSE = 0x302
MCU_FEEDBACK_IDS = (CANID_MCU_CONTROL, CANID_MCU_HEARTBEAT,
                    CANID_MCU_DIAG, CANID_MCU_E2E_DIAG)
CAN_SFF_MASK = 0x7FF
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
PROTOCOL_VERSION = 0x03  # v3: session 由 MCU 自驱授权（2026-07 重构）；CRC/序列/字段编码与 v2 兼容
SRC_PRIMARY = 1
SRC_WATCHDOG = 9
SYS_MODE_INIT = 0
SYS_MODE_STANDBY = 1
SYS_MODE_FAULT_LOCK = 7
SAFE_BRAKE = 1.0
RECOVERY_FRAMES = 3

CAN_FRAME = struct.Struct('=IB3x8s')
CAN_FILTER = struct.Struct('=II')


def socketcan_feedback_filters():
    """Return exact Linux CAN_RAW filters for MCU standard data frames."""
    mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG
    return b''.join(CAN_FILTER.pack(can_id, mask)
                    for can_id in MCU_FEEDBACK_IDS)


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (((value << 1) ^ 0x31) if value & 0x80 else
                     (value << 1)) & 0xFF
    return value


def frame_crc(can_id, data):
    return crc8(bytes((can_id & 0xFF, (can_id >> 8) & 0xFF)) + data[:7])


def encode_fault_injection(command, parameter, sequence):
    """Encode authoritative CAN v3 0x301 payload."""
    if command not in range(10):
        raise ValueError('fault_command_out_of_range')
    if parameter not in range(256) or sequence not in range(256):
        raise ValueError('fault_parameter_or_sequence_out_of_range')
    data = bytearray(8)
    data[0] = command
    data[1] = parameter
    data[5] = sequence
    data[7] = frame_crc(CANID_FAULT_INJECT, data)
    return bytes(data)


def decode_frame(can_id, data):
    if can_id not in (CANID_MCU_CONTROL, CANID_MCU_HEARTBEAT,
                       CANID_MCU_DIAG, CANID_MCU_E2E_DIAG):
        raise ValueError('unsupported_can_id')
    if len(data) != 8:
        raise ValueError('invalid_dlc')
    if data[7] != frame_crc(can_id, data):
        raise ValueError('crc_or_data_id')
    if can_id == CANID_MCU_CONTROL:
        steer_raw, accel_raw = struct.unpack_from('<hh', data)
        if data[4] > 100 or data[5] > 100 or (data[4] and data[5]):
            raise ValueError('invalid_control_payload')
        return {'kind': 'control', 'steer': steer_raw * 0.01 / 30.0,
                'accel_mps2': accel_raw * 0.001,
                'throttle': data[4] * 0.01, 'brake': data[5] * 0.01,
                'seq': data[6]}
    if can_id == CANID_MCU_HEARTBEAT:
        return {'kind': 'heartbeat', 'state': data[0],
                'active_source': data[1], 'safety_flags': data[2],
                'fault_level': data[3], 'seq': data[4],
                'alive': data[5], 'loop_load_pct': data[6]}
    if can_id == CANID_MCU_E2E_DIAG:
        return {'kind': 'e2e', 'protocol_flags': data[2],
                'can_recovery_count': data[3], 'self_test_mask': data[4],
                'test_build': bool(data[5] & 1),
                'protocol_version': data[6]}
    return {'kind': 'diag', 'fault_code': data[0] | (data[1] << 8),
            'reset_reason': data[2], 'primary_timeout': data[3],
            'backup_timeout': data[4], 'crc_errors': data[5],
            'loop_overruns': data[6]}


def sequence_forward(previous, current):
    delta = (current - previous) & 0xFF
    return 0 < delta <= 127


class McuFeedbackGuard:
    """Authenticate feedback and expose a fail-closed CARLA actuation snapshot."""

    def __init__(self, feedback_timeout_s=0.1, heartbeat_timeout_s=0.2,
                 e2e_timeout_s=0.5, clock=time.monotonic):
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.e2e_timeout_s = float(e2e_timeout_s)
        self.clock = clock
        self.control = None
        self.control_rx = 0.0
        self.heartbeat = None
        self.heartbeat_rx = 0.0
        self.e2e = None
        self.e2e_rx = 0.0
        self.last_seq = None
        self.invalid_latched = True
        self.recovery_frames = 0
        self.invalid_count = 0
        self.valid_count = 0
        self.lock = threading.Lock()

    def feed(self, can_id, data):
        now = self.clock()
        try:
            decoded = decode_frame(can_id, bytes(data))
        except (TypeError, ValueError):
            with self.lock:
                self.invalid_latched = True
                self.recovery_frames = 0
                self.invalid_count += 1
            return False
        with self.lock:
            kind = decoded['kind']
            if kind == 'control':
                if self.last_seq is not None and not sequence_forward(
                        self.last_seq, decoded['seq']):
                    self.invalid_latched = True
                    self.recovery_frames = 0
                    self.invalid_count += 1
                    return False
                self.last_seq = decoded['seq']
                self.control = decoded
                self.control_rx = now
                self.valid_count += 1
                if self._health_valid(now):
                    self.recovery_frames += 1
                    if self.recovery_frames >= RECOVERY_FRAMES:
                        self.invalid_latched = False
            elif kind == 'heartbeat':
                self.heartbeat = decoded
                self.heartbeat_rx = now
            elif kind == 'e2e':
                self.e2e = decoded
                self.e2e_rx = now
                if decoded['protocol_version'] != PROTOCOL_VERSION:
                    self.invalid_latched = True
                    self.recovery_frames = 0
                    self.invalid_count += 1
            return True

    def _health_valid(self, now):
        if self.heartbeat is None or self.e2e is None:
            return False
        if now - self.heartbeat_rx > self.heartbeat_timeout_s:
            return False
        if now - self.e2e_rx > self.e2e_timeout_s:
            return False
        if self.e2e['protocol_version'] != PROTOCOL_VERSION:
            return False
        if self.heartbeat['active_source'] not in (SRC_PRIMARY, SRC_WATCHDOG):
            return False
        if self.heartbeat['state'] in (SYS_MODE_INIT, SYS_MODE_STANDBY):
            return False
        return True

    def current(self):
        now = self.clock()
        with self.lock:
            age = now - self.control_rx if self.control is not None else math.inf
            healthy = self.control is not None and age <= self.feedback_timeout_s and \
                self._health_valid(now) and not self.invalid_latched
            if not healthy:
                if self.control is not None and age > self.feedback_timeout_s:
                    self.invalid_latched = True
                    self.recovery_frames = 0
                return {'throttle': 0.0, 'brake': SAFE_BRAKE, 'steer': 0.0,
                        'stale': age > self.feedback_timeout_s,
                        'invalid_latched': self.invalid_latched,
                        'invalid_count': self.invalid_count, 'age_s': age}
            return {'throttle': self.control['throttle'],
                    'brake': self.control['brake'],
                    'steer': max(-1.0, min(1.0, self.control['steer'])),
                    'stale': False, 'invalid_latched': False,
                    'invalid_count': self.invalid_count, 'age_s': age}


class SocketCanReceiver:
    def __init__(self, interface, feedback_timeout_s=0.1):
        if not hasattr(socket, 'PF_CAN'):
            raise RuntimeError('SocketCAN is only available on Linux')
        if not math.isfinite(feedback_timeout_s) or feedback_timeout_s <= 0.0:
            raise ValueError('feedback_timeout_s must be finite and positive')
        self.guard = McuFeedbackGuard(feedback_timeout_s=feedback_timeout_s)
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.setsockopt(getattr(socket, 'SOL_CAN_RAW', 101),
                               getattr(socket, 'CAN_RAW_FILTER', 1),
                               socketcan_feedback_filters())
        self.socket.bind((interface,))
        self.socket.settimeout(0.2)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name='socketcan-rx', daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                while True:
                    packet = self.socket.recv(CAN_FRAME.size)
                    if len(packet) != CAN_FRAME.size:
                        self.guard.feed(0, b'')
                        continue
                    can_id, dlc, data = CAN_FRAME.unpack(packet)
                    if can_id & 0xE0000000 or dlc != 8:
                        self.guard.feed(0, b'')
                        continue
                    self.guard.feed(can_id & 0x7FF, data)
            except socket.timeout:
                continue
            except OSError:
                break

    def current(self):
        return self.guard.current()

    def send_fault_injection(self, command, parameter, sequence):
        data = encode_fault_injection(command, parameter, sequence)
        self.socket.send(CAN_FRAME.pack(CANID_FAULT_INJECT, 8, data))

    def close(self):
        self.stop_event.set()
        try:
            self.socket.close()
        finally:
            self.thread.join(timeout=1.0)


class CanalystReceiver:
    """Receive MCU feedback through python-can's userspace CANalyst-II backend."""

    def __init__(self, device=0, channel=1, bitrate=500000,
                 feedback_timeout_s=0.1, bus_factory=None):
        if not math.isfinite(feedback_timeout_s) or feedback_timeout_s <= 0.0:
            raise ValueError('feedback_timeout_s must be finite and positive')
        if device < 0:
            raise ValueError('CANalyst-II device index must be non-negative')
        if channel not in (0, 1):
            raise ValueError('CANalyst-II channel must be 0 or 1')
        if bitrate <= 0:
            raise ValueError('CANalyst-II bitrate must be positive')
        if bus_factory is None:
            try:
                import can
            except ImportError as error:
                raise RuntimeError(
                    'python-can/canalystii is unavailable; install python3-can, '
                    'python3-usb and canalystii') from error
            bus_factory = can.Bus
            self._can_module = can
        else:
            self._can_module = None
        filters = [{'can_id': can_id, 'can_mask': CAN_SFF_MASK,
                    'extended': False} for can_id in MCU_FEEDBACK_IDS]
        self.guard = McuFeedbackGuard(feedback_timeout_s=feedback_timeout_s)
        self.bus = bus_factory(
            interface='canalystii', device=device, channel=channel,
            bitrate=bitrate, can_filters=filters, rx_queue_size=4096)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run, name='canalystii-rx', daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                while True:
                    message = self.bus.recv(timeout=0.005)
                    if message is None:
                        break
                    if (message.is_extended_id or message.is_remote_frame or
                            message.dlc != 8 or
                            message.arbitration_id not in MCU_FEEDBACK_IDS):
                        self.guard.feed(0, b'')
                        continue
                    self.guard.feed(message.arbitration_id, message.data)
            except Exception:
                self.guard.feed(0, b'')
                break

    def current(self):
        return self.guard.current()

    def send_fault_injection(self, command, parameter, sequence):
        data = encode_fault_injection(command, parameter, sequence)
        if self._can_module is not None:
            message = self._can_module.Message(
                arbitration_id=CANID_FAULT_INJECT, is_extended_id=False,
                data=data)
        else:
            # Test bus factories accept a minimal message-shaped object.
            message = type('CanMessage', (), {
                'arbitration_id': CANID_FAULT_INJECT,
                'is_extended_id': False, 'data': data, 'dlc': 8})()
        self.bus.send(message)

    def close(self):
        self.stop_event.set()
        try:
            self.bus.shutdown()
        finally:
            self.thread.join(timeout=1.0)
