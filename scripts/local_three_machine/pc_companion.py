#!/usr/bin/env python3
"""PC-side map publisher and request-scoped CAN v3 fault relay for local HIL."""

import argparse
import json
import socket
import struct
import threading
import time
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from adas_msgs.msg import LaneConnection, LaneGraph, MapLane
from geometry_msgs.msg import Pose
from std_msgs.msg import String

CAN_FRAME = struct.Struct('=IB3x8s')
CAN_FILTER = struct.Struct('=II')
CANID_FAULT_INJECT = 0x301
CANID_FAULT_RESPONSE = 0x302


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


def uuid4_valid(value):
    try:
        parsed = uuid.UUID(str(value))
        return parsed.version == 4 and str(parsed) == str(value).lower()
    except (ValueError, AttributeError, TypeError):
        return False


def sequence_forward(previous, current):
    return previous is None or 0 < ((current - previous) & 0xFF) <= 127


class PcCompanion(Node):
    def __init__(self, interface, ack_timeout):
        super().__init__('local_three_machine_pc')
        self.ack_timeout = ack_timeout
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(
            socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
            CAN_FILTER.pack(CANID_FAULT_RESPONSE, 0xC00007FF))
        self.sock.bind((interface,))
        self.sock.settimeout(0.05)
        self.sequence = 0
        self.last_ack_sequence = None
        self.busy = threading.Lock()
        self.cache = {}
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(LaneGraph, '/adas/map/lane_graph', qos)
        self.ack_pub = self.create_publisher(
            String, '/adas/_debug/fault_inject_ack', 10)
        self.create_subscription(
            String, '/adas/_debug/fault_inject_cmd', self.on_fault, 10)
        self.create_timer(0.5, self.publish_map)
        self.map_published = False
        self.get_logger().info(
            'local PC companion ready on %s (CAN v3 fault relay)' % interface)

    def publish_map(self):
        if self.map_published and self.count_subscribers('/adas/map/lane_graph'):
            return
        graph = LaneGraph()
        graph.header.stamp = self.get_clock().now().to_msg()
        graph.header.frame_id = 'map'
        graph.map_id = 'local-three-machine-track'
        graph.map_hash = 'local-three-machine-v1'
        lane = MapLane()
        lane.id = 1
        lane.speed_limit_mps = 12.0
        lane.junction = False
        for x in range(0, 301, 5):
            pose = Pose()
            pose.position.x = float(x)
            pose.orientation.w = 1.0
            lane.centerline.append(pose)
        graph.lanes.append(lane)
        self.map_pub.publish(graph)
        self.map_published = True

    def publish_ack(self, request_id, command, accepted, detail, **extra):
        msg = String()
        payload = {
            'request_id': request_id, 'cmd': command,
            'accepted': accepted, 'detail': detail,
            'source': 'local_three_machine_pc', **extra,
        }
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.ack_pub.publish(msg)
        if request_id:
            self.cache[request_id] = msg.data

    def on_fault(self, msg):
        try:
            payload = json.loads(msg.data)
            request_id = str(payload.get('request_id', '')).lower()
            command = int(payload.get('cmd', -1))
            parameter = int(payload.get('param', 0))
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.publish_ack('', -1, False, 'invalid JSON: %s' % error)
            return
        if request_id in self.cache:
            cached = String()
            cached.data = self.cache[request_id]
            self.ack_pub.publish(cached)
            return
        if not uuid4_valid(request_id):
            self.publish_ack(request_id, command, False, 'request_id must be UUID v4')
            return
        if command not in range(10) or parameter not in range(256):
            self.publish_ack(request_id, command, False, 'cmd/param out of range')
            return
        if not self.busy.acquire(blocking=False):
            self.publish_ack(request_id, command, False, 'fault relay busy')
            return
        threading.Thread(
            target=self.relay_fault,
            args=(request_id, command, parameter), daemon=True).start()

    def relay_fault(self, request_id, command, parameter):
        try:
            # Drain delayed responses so an old 0x302 cannot complete a new request.
            while True:
                try:
                    self.sock.recv(CAN_FRAME.size, socket.MSG_DONTWAIT)
                except (BlockingIOError, socket.timeout):
                    break
            self.sequence = (self.sequence + 1) & 0xFF
            data = bytearray(8)
            data[0] = command
            data[1] = parameter
            data[5] = self.sequence
            data[7] = frame_crc(CANID_FAULT_INJECT, data)
            self.sock.send(CAN_FRAME.pack(CANID_FAULT_INJECT, 8, bytes(data)))
            deadline = time.monotonic() + self.ack_timeout
            while time.monotonic() < deadline:
                try:
                    raw = self.sock.recv(CAN_FRAME.size)
                except socket.timeout:
                    continue
                can_id, dlc, response = CAN_FRAME.unpack(raw)
                can_id &= 0x7FF
                if can_id != CANID_FAULT_RESPONSE or dlc != 8:
                    continue
                if response[7] != frame_crc(can_id, response):
                    continue
                if response[0] != command or response[1] != parameter:
                    continue
                if not sequence_forward(self.last_ack_sequence, response[5]):
                    continue
                self.last_ack_sequence = response[5]
                self.publish_ack(
                    request_id, command, True, 'MCU 0x302 acknowledged',
                    param=parameter, system_state=response[2],
                    fault_level=response[3], alive=response[4],
                    sequence=response[5], fault_code_low=response[6],
                    can_id='0x302', dlc=8, crc_valid=True)
                return
            self.publish_ack(
                request_id, command, False, 'MCU 0x302 response timeout',
                param=parameter, timed_out=True)
        except OSError as error:
            self.publish_ack(request_id, command, False, 'CAN error: %s' % error)
        finally:
            self.busy.release()

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--interface', default='vcan0')
    parser.add_argument('--ack-timeout', type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    node = PcCompanion(args.interface, args.ack_timeout)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
