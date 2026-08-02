#!/usr/bin/env python3
"""Record a running ADAS SIL scenario into a MATLAB-friendly CSV file.

This tool is intentionally outside the control chain: it only subscribes to
existing ROS2 topics and periodically writes the latest observed state.
"""

import argparse
import csv
import math
import time

import rclpy
from adas_msgs.msg import AebStatus, BehaviorState, GateStatus, LaneState, TrackedObjectArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class SilRecorder(Node):
    def __init__(self):
        super().__init__('sil_plot_recorder')
        self.data = {
            'speed_mps': math.nan, 'x_m': math.nan, 'y_m': math.nan,
            'lateral_offset_m': math.nan, 'heading_error_rad': math.nan,
            'curvature_1pm': math.nan, 'lead_gap_m': math.nan,
            'lead_speed_mps': math.nan, 'behavior_state': math.nan,
            'aeb_state': math.nan, 'aeb_ttc_s': math.nan,
            'gate_source': math.nan, 'primary_gate_source': math.nan,
            'backup_gate_source': math.nan,
        }
        self.create_subscription(Odometry, '/adas/localization/kinematic_state',
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaneState, '/adas/perception/lane_state',
                                 self.on_lane, qos_profile_sensor_data)
        self.create_subscription(TrackedObjectArray, '/adas/perception/objects',
                                 self.on_objects, qos_profile_sensor_data)
        self.create_subscription(BehaviorState, '/adas/planning/behavior',
                                 self.on_behavior, 10)
        self.create_subscription(AebStatus, '/adas/control/aeb/status', self.on_aeb, 10)
        # Gate status is latched so a recorder joining just after activation
        # still sees the selected source rather than an all-NaN trace.
        gate_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(GateStatus, '/adas/control/gate/status', self.on_gate, gate_qos)
        self.create_subscription(GateStatus, '/primary/adas/control/gate/status',
                                 self.on_primary_gate, gate_qos)
        self.create_subscription(GateStatus, '/backup/adas/control/gate/status',
                                 self.on_backup_gate, gate_qos)

    def on_odom(self, msg):
        self.data['speed_mps'] = msg.twist.twist.linear.x
        self.data['x_m'] = msg.pose.pose.position.x
        self.data['y_m'] = msg.pose.pose.position.y

    def on_lane(self, msg):
        self.data['lateral_offset_m'] = msg.lateral_offset
        self.data['heading_error_rad'] = msg.heading_error
        self.data['curvature_1pm'] = msg.curvature

    def on_objects(self, msg):
        self.data['lead_gap_m'] = msg.primary_lead_gap_m if msg.primary_lead_id >= 0 else math.nan
        self.data['lead_speed_mps'] = (
            msg.primary_lead_speed_mps if msg.primary_lead_id >= 0 else math.nan)

    def on_behavior(self, msg):
        self.data['behavior_state'] = msg.state

    def on_aeb(self, msg):
        self.data['aeb_state'] = msg.state
        self.data['aeb_ttc_s'] = msg.ttc_s

    def on_gate(self, msg):
        self.data['gate_source'] = msg.selected_source

    def on_primary_gate(self, msg):
        self.data['primary_gate_source'] = msg.selected_source

    def on_backup_gate(self, msg):
        self.data['backup_gate_source'] = msg.selected_source


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration-s', type=float, required=True)
    parser.add_argument('--rate-hz', type=float, default=20.0)
    args = parser.parse_args()
    if args.duration_s <= 0.0 or args.rate_hz <= 0.0:
        raise SystemExit('duration-s and rate-hz must be positive')

    rclpy.init()
    node = SilRecorder()
    columns = ['t_s', *node.data.keys()]
    start = time.monotonic()
    next_sample = start
    period = 1.0 / args.rate_hz
    with open(args.output, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        while time.monotonic() - start < args.duration_s:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_sample:
                writer.writerow({'t_s': now - start, **node.data})
                next_sample += period
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
