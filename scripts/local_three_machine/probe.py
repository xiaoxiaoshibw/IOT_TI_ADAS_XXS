#!/usr/bin/env python3
"""Small request/readiness probe used while the orchestrator restarts roles."""

import argparse
import json
import threading
import time
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from adas_msgs.srv import SetNavigationGoal


class Probe(Node):
    def __init__(self):
        super().__init__('local_three_machine_recovery_probe')
        self.acks = {}
        self.publisher = self.create_publisher(
            String, '/adas/_debug/fault_inject_cmd', 10)
        self.create_subscription(
            String, '/adas/_debug/fault_inject_ack', self.ack, 10)
        self.goal = self.create_client(
            SetNavigationGoal, '/adas/navigation/set_goal')

    def ack(self, msg):
        try:
            payload = json.loads(msg.data)
            self.acks[payload.get('request_id', '')] = payload
        except json.JSONDecodeError:
            pass

    def fault(self, timeout):
        discovery_deadline = time.monotonic() + timeout
        while (self.publisher.get_subscription_count() == 0 and
               time.monotonic() < discovery_deadline):
            time.sleep(0.02)
        request_id = str(uuid.uuid4())
        msg = String()
        msg.data = json.dumps(
            {'request_id': request_id, 'cmd': 0, 'param': 0}, separators=(',', ':'))
        self.publisher.publish(msg)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and request_id not in self.acks:
            time.sleep(0.02)
        return self.acks.get(request_id)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--nav', choices=['available', 'unavailable'])
    parser.add_argument('--fault', choices=['success', 'timeout'])
    parser.add_argument('--timeout', type=float, default=2.0)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    ok = True
    detail = []
    try:
        if args.nav:
            ready = node.goal.wait_for_service(timeout_sec=args.timeout)
            ok &= ready == (args.nav == 'available')
            detail.append('nav_ready=%s' % ready)
        if args.fault:
            ack = node.fault(args.timeout)
            success = bool(ack and ack.get('accepted'))
            ok &= success == (args.fault == 'success')
            detail.append('fault_ack=%s' % success)
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(' '.join(detail))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
