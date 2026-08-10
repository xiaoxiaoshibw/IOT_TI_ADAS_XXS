#!/usr/bin/env python3
"""Machine-readable acceptance checks for the local three-machine HIL."""

import argparse
from collections import defaultdict
import json
import os
import signal
import socket
import struct
import threading
import time
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from adas_msgs.msg import (ActuationCommand, BehaviorState, GateStatus, LaneState,
                           McuStatus, NavigationStatus, SafetyStatus)
from adas_msgs.srv import CancelNavigation, SetNavigationGoal
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

CAN_FRAME = struct.Struct('=IB3x8s')
CAN_FLAGS = 0xE0000000
REQUIRED_CAN = (0x101, 0x102, 0x104, 0x201, 0x202, 0x203, 0x204, 0x206)
SEQ_BYTE = {0x101: 5, 0x102: 6, 0x104: 6, 0x201: 6, 0x202: 4}


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (((value << 1) ^ 0x31) if value & 0x80 else value << 1) & 0xFF
    return value


def valid_crc(can_id, data):
    return data[7] == crc8(bytes((can_id & 0xFF, can_id >> 8)) + data[:7])


def forward(previous, current):
    return 0 < ((current - previous) & 0xFF) <= 127


class CanCapture:
    def __init__(self, interface):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.settimeout(0.1)
        self.frames = defaultdict(list)
        self.bad = []
        self.stop = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop:
            try:
                raw = self.sock.recv(CAN_FRAME.size)
            except socket.timeout:
                continue
            except OSError:
                return
            raw_id, dlc, data = CAN_FRAME.unpack(raw)
            can_id = raw_id & 0x7FF
            if raw_id & CAN_FLAGS or dlc != 8 or not valid_crc(can_id, data):
                self.bad.append((raw_id, dlc, data.hex()))
            self.frames[can_id].append((time.monotonic(), data))

    def close(self):
        self.stop = True
        self.sock.close()
        self.thread.join(timeout=1.0)


class Checker(Node):
    def __init__(self):
        super().__init__('local_three_machine_checker')
        self.times = defaultdict(list)
        self.last = {}
        self.odom_samples = []
        self.fault_acks = {}
        sensor = QoSProfile(depth=20)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        transient = QoSProfile(depth=10)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        topics = [
            (Odometry, '/adas/localization/kinematic_state', sensor),
            (LaneState, '/adas/perception/lane_state', sensor),
            (BehaviorState, '/adas/planning/behavior', 20),
            (GateStatus, '/adas/control/gate/status', transient),
            (ActuationCommand, '/adas/vehicle/actuation_cmd', 20),
            (McuStatus, '/adas/mcu/status', 20),
            (ActuationCommand, '/adas/mcu/actuation_feedback', 20),
            (SafetyStatus, '/adas/system/safety_status', transient),
            (NavigationStatus, '/adas/navigation/status', transient),
            (Path, '/adas/planning/global_route', transient),
        ]
        self.subs = []
        for msg_type, topic, qos in topics:
            self.subs.append(self.create_subscription(
                msg_type, topic, lambda msg, name=topic: self.on_topic(name, msg), qos))
        self.fault_pub = self.create_publisher(
            String, '/adas/_debug/fault_inject_cmd', 10)
        self.subs.append(self.create_subscription(
            String, '/adas/_debug/fault_inject_ack', self.on_ack, 10))
        self.goal = self.create_client(SetNavigationGoal, '/adas/navigation/set_goal')
        self.cancel = self.create_client(CancelNavigation, '/adas/navigation/cancel_goal')

    def on_topic(self, name, msg):
        self.times[name].append(time.monotonic())
        self.last[name] = msg
        if name == '/adas/localization/kinematic_state':
            self.odom_samples.append((
                time.monotonic(), float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y), float(msg.twist.twist.linear.x)))

    def on_ack(self, msg):
        try:
            payload = json.loads(msg.data)
            self.fault_acks[payload.get('request_id', '')] = payload
        except json.JSONDecodeError:
            pass

    def send_fault(self, command, timeout=2.0):
        request_id = str(uuid.uuid4())
        msg = String()
        msg.data = json.dumps({
            'request_id': request_id, 'cmd': command, 'param': 0,
            'source': 'local_three_machine_checker'}, separators=(',', ':'))
        self.fault_pub.publish(msg)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and request_id not in self.fault_acks:
            time.sleep(0.02)
        return request_id, self.fault_acks.get(request_id)


class Results:
    def __init__(self):
        self.items = []

    def add(self, name, ok, detail):
        self.items.append({'name': name, 'status': 'PASS' if ok else 'FAIL',
                           'detail': detail})
        print('[%s] %s%s' % ('PASS' if ok else 'FAIL', name,
                             '' if not detail else ': ' + detail), flush=True)

    @property
    def ok(self):
        return all(item['status'] == 'PASS' for item in self.items)


def wait_future(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--interface', default='vcan0')
    parser.add_argument('--report', required=True)
    parser.add_argument('--mcu-pid', type=int, required=True)
    parser.add_argument('--fault-test', action='store_true')
    parser.add_argument('--dangerous-fault-test', action='store_true')
    parser.add_argument('--navigation-test', action='store_true')
    parser.add_argument('--carla', action='store_true')
    parser.add_argument('--carla-host', default='127.0.0.1')
    parser.add_argument('--carla-port', type=int, default=2000)
    parser.add_argument('--scenario', default='baseline')
    args = parser.parse_args()

    results = Results()
    capture = CanCapture(args.interface)
    rclpy.init()
    node = Checker()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        pc_node = '/carla_bridge' if args.carla else '/local_three_machine_pc'
        vehicle_node = set() if args.carla else {'/sim_vehicle'}
        expected_nodes = {pc_node, '/can_gateway', '/global_planner',
                          '/behavior_planner', '/trajectory_planner'} | vehicle_node
        deadline = time.monotonic() + 45.0
        names = set()
        while time.monotonic() < deadline:
            names = set(node.get_node_names_and_namespaces())
            names = {(ns.rstrip('/') + '/' + name).replace('//', '/')
                     for name, ns in names}
            if expected_nodes <= names:
                break
            time.sleep(0.2)
        missing = sorted(expected_nodes - names)
        results.add('Orin stack', not missing,
                    'all key nodes ready' if not missing else 'missing ' + ','.join(missing))

        # Measure continuity over a common window, not merely one retained sample.
        start = time.monotonic()
        time.sleep(4.0)
        required_topics = [
            '/adas/localization/kinematic_state', '/adas/perception/lane_state',
            '/adas/planning/behavior', '/adas/control/gate/status',
            '/adas/vehicle/actuation_cmd', '/adas/mcu/status',
            '/adas/mcu/actuation_feedback', '/adas/system/safety_status',
            '/adas/navigation/status']
        counts = {topic: sum(stamp >= start for stamp in node.times[topic])
                  for topic in required_topics}
        continuous = all(count >= 2 for count in counts.values())
        results.add('ROS closed loop', continuous, json.dumps(counts, sort_keys=True))

        if args.carla:
            carla_ok = False
            carla_detail = ''
            try:
                import carla
                client = carla.Client(args.carla_host, args.carla_port)
                client.set_timeout(3.0)
                world = client.get_world()
                map_name = world.get_map().name
                actors = world.get_actors()
                heroes = [actor for actor in actors
                          if actor.attributes.get('role_name') == 'hero']
                leads = [actor for actor in actors
                         if actor.attributes.get('role_name') == 'lead']
                needs_lead = args.scenario in ('acc', 'aeb', 'overtake')
                actor_ok = len(heroes) == 1 and (not needs_lead or len(leads) >= 1)

                deadline = time.monotonic() + 20.0
                distance = 0.0
                max_speed = 0.0
                while time.monotonic() < deadline:
                    samples = list(node.odom_samples)
                    if len(samples) >= 2:
                        x0, y0 = samples[0][1], samples[0][2]
                        distance = max(
                            ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
                            for _, x, y, _ in samples)
                        max_speed = max(abs(speed) for *_, speed in samples)
                    if distance >= 1.0 and max_speed >= 0.5:
                        break
                    time.sleep(0.1)
                carla_ok = ('Town04' in map_name and actor_ok and
                            distance >= 1.0 and max_speed >= 0.5)
                carla_detail = ('map=%s hero=%d lead=%d displacement=%.2fm '
                                'max_speed=%.2fm/s' %
                                (map_name, len(heroes), len(leads), distance, max_speed))
            except Exception as error:
                carla_detail = repr(error)
            results.add('CARLA vehicle dynamics', carla_ok, carla_detail)

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and any(
                len(capture.frames[can_id]) < 3 for can_id in REQUIRED_CAN):
            time.sleep(0.1)
        missing_can = [hex(can_id) for can_id in REQUIRED_CAN
                       if len(capture.frames[can_id]) < 3]
        seq_errors = []
        for can_id, index in SEQ_BYTE.items():
            values = [data[index] for _, data in capture.frames[can_id]]
            if len(values) >= 3 and not all(forward(a, b) for a, b in zip(values, values[1:])):
                seq_errors.append(hex(can_id))
        v3 = (capture.frames[0x204] and capture.frames[0x204][-1][1][6] == 3 and
              capture.frames[0x206] and capture.frames[0x206][-1][1][0] == 3)
        active = any(data[1] == 4 for _, data in capture.frames[0x206]) and any(
            data[0] == 2 for _, data in capture.frames[0x202])
        controls = [data for _, data in capture.frames[0x201]]
        legal = bool(controls) and all(
            data[4] <= 100 and data[5] <= 100 and not (data[4] and data[5])
            for data in controls)
        can_ok = not missing_can and not capture.bad and not seq_errors and v3 and active and legal
        results.add('CAN v3 CRC/sequence', can_ok,
                    'missing=%s bad=%d seq=%s v3=%s active=%s legal=%s' %
                    (missing_can, len(capture.bad), seq_errors, v3, active, legal))

        if args.navigation_test:
            services = node.goal.wait_for_service(5.0) and node.cancel.wait_for_service(5.0)
            nav_ok = services
            detail = []
            if services:
                request_id = str(uuid.uuid4())
                req = SetNavigationGoal.Request()
                req.request_id = request_id
                req.goal.header.frame_id = 'map'
                req.goal.pose.position.x = 100.0
                req.goal.pose.orientation.w = 1.0
                first = wait_future(node.goal.call_async(req), 5.0)
                duplicate = wait_future(node.goal.call_async(req), 5.0)
                second_req = SetNavigationGoal.Request()
                second_req.request_id = str(uuid.uuid4())
                second_req.goal = req.goal
                second_req.goal.pose.position.x = 120.0
                second = wait_future(node.goal.call_async(second_req), 5.0)
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    route = node.last.get('/adas/planning/global_route')
                    status = node.last.get('/adas/navigation/status')
                    if route and route.poses and status and status.state in (2, 3):
                        break
                    time.sleep(0.05)
                route = node.last.get('/adas/planning/global_route')
                status = node.last.get('/adas/navigation/status')
                cancel_req = CancelNavigation.Request()
                cancel_req.request_id = str(uuid.uuid4())
                cancel_req.goal_id = first.goal_id if first else ''
                canceled = wait_future(node.cancel.call_async(cancel_req), 5.0)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    status = node.last.get('/adas/navigation/status')
                    route = node.last.get('/adas/planning/global_route')
                    if status and status.state == 6 and route is not None and not route.poses:
                        break
                    time.sleep(0.05)
                nav_ok = bool(
                    first and first.accepted and first.request_id == request_id and
                    duplicate and duplicate.accepted and duplicate.goal_id == first.goal_id and
                    second and not second.accepted and route is not None and not route.poses and
                    status and status.state == 6 and canceled and canceled.accepted and
                    canceled.request_id == cancel_req.request_id)
                detail.append('set=%s duplicate=%s second_locked=%s cancel=%s' %
                              (bool(first and first.accepted), bool(duplicate and duplicate.accepted),
                               bool(second and not second.accepted), bool(canceled and canceled.accepted)))
            results.add('Navigation set/cancel', nav_ok, '; '.join(detail))

        if args.fault_test:
            fault_ok = True
            fault_detail = []
            commands = [1, 3, 7, 0]
            if args.dangerous_fault_test:
                commands = [1, 3, 7, 2, 5, 6, 0]
            for command in commands:
                request_id, ack = node.send_fault(command)
                valid_id = uuid.UUID(request_id).version == 4
                ok = bool(ack and ack.get('accepted') and
                          ack.get('request_id') == request_id and
                          ack.get('cmd') == command and ack.get('can_id') == '0x302' and
                          ack.get('dlc') == 8 and ack.get('crc_valid'))
                fault_ok &= ok
                fault_detail.append('%d=%s' % (command, 'ack' if ok else str(ack)))
                time.sleep(0.6)
            results.add('Fault 0x301 → MCU → 0x302', fault_ok and valid_id,
                        ', '.join(fault_detail))

            # Pause the actual host runner: request must time out, resume must recover.
            os.killpg(args.mcu_pid, signal.SIGSTOP)
            try:
                _, timed_out = node.send_fault(0, timeout=2.5)
            finally:
                os.killpg(args.mcu_pid, signal.SIGCONT)
            time.sleep(0.3)
            _, recovered = node.send_fault(0, timeout=2.5)
            timeout_ok = bool(timed_out and not timed_out.get('accepted') and
                              timed_out.get('timed_out') and recovered and
                              recovered.get('accepted'))
            results.add('MCU/0x302 timeout recovery', timeout_ok,
                        'timeout=%s recovered=%s' % (bool(timed_out), bool(recovered)))

    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)
        spin.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
        capture.close()

    try:
        with open(args.report, encoding='utf-8') as stream:
            report = json.load(stream)
    except (OSError, json.JSONDecodeError):
        report = {'schema_version': 1, 'checks': []}
    report.setdefault('checks', []).extend(results.items)
    report['overall'] = ('PASS' if all(item.get('status') == 'PASS'
                                      for item in report['checks']) else 'FAIL')
    report['finished_at'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    with open(args.report, 'w', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    return 0 if results.ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
