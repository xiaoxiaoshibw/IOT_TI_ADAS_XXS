#!/usr/bin/env python3
"""Machine-readable acceptance checks for the local MIL hardware simulation."""

import argparse
from collections import defaultdict
import json
import math
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
                           McuStatus, NavigationStatus, SafetyStatus,
                           TrackedObjectArray)
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
        self.lane_samples = []
        self.behavior_samples = []
        self.object_frames = defaultdict(list)
        self.fault_acks = {}
        sensor = QoSProfile(depth=20)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        transient = QoSProfile(depth=10)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        topics = [
            (Odometry, '/adas/localization/kinematic_state', sensor),
            (LaneState, '/adas/perception/lane_state', sensor),
            (TrackedObjectArray, '/adas/perception/objects_raw', sensor),
            (TrackedObjectArray, '/adas/perception/objects', sensor),
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
        stamp = time.monotonic()
        self.times[name].append(stamp)
        self.last[name] = msg
        if name == '/adas/localization/kinematic_state':
            self.odom_samples.append((
                stamp, float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y), float(msg.twist.twist.linear.x)))
        elif name == '/adas/perception/lane_state':
            self.lane_samples.append((
                stamp, float(msg.lateral_offset), bool(msg.left_lane_available),
                bool(msg.right_lane_available)))
        elif name == '/adas/planning/behavior':
            self.behavior_samples.append((stamp, int(msg.target_lane)))
        elif name in ('/adas/perception/objects_raw', '/adas/perception/objects'):
            self.object_frames[name].append((
                stamp, tuple(int(obj.id) for obj in msg.objects),
                int(msg.primary_lead_id),
                tuple((float(obj.pose.pose.position.x),
                       float(obj.pose.pose.position.y)) for obj in msg.objects)))

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
    parser.add_argument('--run-id', required=True,
                        help='canonical UUID v4 shared by GUI/bridge/planner')
    parser.add_argument('--scenario-file', default='')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    scenario = None
    if args.scenario_file:
        try:
            with open(args.scenario_file, encoding='utf-8') as stream:
                scenario = json.load(stream)
        except (OSError, json.JSONDecodeError) as error:
            print('[FAIL] Scenario acceptance: %s' % error, flush=True)
            return 2

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

        if scenario is not None:
            expected_ids = tuple(int(actor['id']) for actor in scenario['actors'])
            raw_frames = [frame for frame in
                          node.object_frames['/adas/perception/objects_raw']
                          if frame[0] >= start]
            exact = (len(raw_frames) >= 20 and all(
                frame[1] == expected_ids and frame[2] == -1
                for frame in raw_frames))
            results.add(
                'Scenario actor IDs/count', exact,
                'id=%s seed=%d expected=%d frames=%d observed=%s primary_raw=%s' % (
                    scenario.get('id'), args.seed, len(expected_ids), len(raw_frames),
                    list(raw_frames[-1][1]) if raw_frames else [],
                    raw_frames[-1][2] if raw_frames else 'missing'))

            gaps = [b[0] - a[0] for a, b in zip(raw_frames, raw_frames[1:])]
            max_gap = max(gaps, default=float('inf'))
            elapsed = (raw_frames[-1][0] - raw_frames[0][0]
                       if len(raw_frames) >= 2 else 0.0)
            average_hz = ((len(raw_frames) - 1) / elapsed if elapsed > 0.0 else 0.0)
            sorted_gaps = sorted(gaps)
            p95_gap = (sorted_gaps[min(len(sorted_gaps) - 1,
                                        int(len(sorted_gaps) * 0.95))]
                       if sorted_gaps else float('inf'))
            # bridge_node publishes lane/object truth on its documented 50 ms
            # slow timer (20 Hz). Judge sustained throughput separately from
            # bounded scheduler jitter and real stalls.
            continuous_objects = (len(raw_frames) >= 20 and average_hz >= 18.0 and
                                  p95_gap <= 0.25 and max_gap <= 0.75)
            results.add('Scenario object continuity', continuous_objects,
                        'frames=%d average_hz=%.1f p95_gap_s=%.3f max_gap_s=%.3f' % (
                            len(raw_frames), average_hz, p95_gap, max_gap))

            min_spacing = float('inf')
            for _, _, _, positions in raw_frames:
                for index, first in enumerate(positions):
                    for second in positions[index + 1:]:
                        min_spacing = min(
                            min_spacing,
                            math.hypot(first[0] - second[0], first[1] - second[1]))
            spacing_ok = len(expected_ids) < 2 or min_spacing >= 2.0
            results.add('Scenario minimum actor spacing', spacing_ok,
                        'minimum_m=%s threshold_m=2.0' % (
                            'n/a' if math.isinf(min_spacing) else '%.2f' % min_spacing))

            tracked = [frame for frame in node.object_frames['/adas/perception/objects']
                       if frame[0] >= start]
            leads = [frame[2] for frame in tracked]
            switches = sum(current != previous for previous, current
                           in zip(leads, leads[1:]))
            stable_primary = len(tracked) >= 2 and switches <= 4
            results.add('Scenario primary lead stability', stable_primary,
                        'frames=%d switches=%d values=%s' % (
                            len(tracked), switches, sorted(set(leads))))

            acceptance = scenario.get('acceptance', {})
            declared_windows = acceptance.get('behavior_aeb_windows', [])
            # Schema v1 currently declares no behavior/AEB windows. Preserve an
            # explicit report item so a future catalog extension cannot be silently ignored.
            windows_ok = not declared_windows
            results.add('Scenario behavior/AEB windows', windows_ok,
                        ('none declared' if not declared_windows else
                         'unsupported declarations=%s' % declared_windows))

        if args.carla:
            lane_samples = [sample for sample in node.lane_samples if sample[0] >= start]
            behavior_samples = [sample for sample in node.behavior_samples
                                if sample[0] >= start]
            invalid_requests = 0
            for behavior_stamp, target_lane in behavior_samples:
                preceding = [sample for sample in lane_samples
                             if 0.0 <= behavior_stamp - sample[0] <= 0.2]
                if not preceding or target_lane == 0:
                    continue
                lane = preceding[-1]
                invalid_requests += int(
                    (target_lane < 0 and not lane[2]) or
                    (target_lane > 0 and not lane[3]))
            max_lateral = max((abs(sample[1]) for sample in lane_samples), default=math.inf)
            lane_change_requested = any(target_lane != 0
                                        for _, target_lane in behavior_samples)
            boundary_ok = bool(lane_samples and behavior_samples and
                               invalid_requests == 0)
            results.add('Lane boundary guard', boundary_ok,
                        ('invalid_requests=%d lane_change_requested=%s '
                         'max_abs_lateral_m=%.3f samples=%d') % (
                            invalid_requests, lane_change_requested,
                            max_lateral, len(lane_samples)))

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
                          if actor.attributes.get('role_name') == 'hero' or
                          actor.attributes.get('role_name', '').endswith(':ego')]
                leads = [actor for actor in actors
                         if actor.attributes.get('role_name') == 'lead' or
                         ':actor:' in actor.attributes.get('role_name', '')]
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
                goal_x, goal_y = 100.0, 0.0
                if args.carla:
                    try:
                        import carla
                        # Reuse the actor/world snapshot already proven healthy
                        # by the dynamics check. A second Client.get_actors()
                        # can observe a transient empty registry at a sync tick
                        # boundary even though the run-scoped ego is alive.
                        if len(heroes) != 1:
                            raise RuntimeError('expected exactly one run-scoped ego')
                        waypoint = world.get_map().get_waypoint(
                            heroes[0].get_location(), project_to_road=True,
                            lane_type=carla.LaneType.Driving)
                        candidates = waypoint.next(120.0) if waypoint else []
                        if not candidates:
                            raise RuntimeError('no reachable waypoint 120m ahead')
                        target = sorted(
                            candidates,
                            key=lambda item: (item.road_id, item.section_id,
                                              item.lane_id))[0].transform.location
                        goal_x, goal_y = float(target.x), -float(target.y)
                    except Exception as error:
                        nav_ok = False
                        detail.append('goal_resolution=%r' % error)
                request_id = str(uuid.uuid4())
                req = SetNavigationGoal.Request()
                req.request_id = request_id
                req.run_id = args.run_id
                req.goal.header.frame_id = 'map'
                req.goal.pose.position.x = goal_x
                req.goal.pose.position.y = goal_y
                req.goal.pose.orientation.w = 1.0
                first = wait_future(node.goal.call_async(req), 5.0)
                duplicate = wait_future(node.goal.call_async(req), 5.0)
                second_req = SetNavigationGoal.Request()
                second_req.request_id = str(uuid.uuid4())
                second_req.run_id = args.run_id
                second_req.goal = req.goal
                second_req.goal.pose.position.x = goal_x + 5.0
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
                cancel_req.run_id = args.run_id
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
