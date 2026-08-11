#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""端到端回环验证：扮演 CARLA 桥（直道运动学模型）↔ 真实 SoC C++ 栈。

用途：没有 CARLA 时验证「话题契约 + QoS + carla.launch.py 栈组装」。
先起栈：  ros2 launch adas_launch carla.launch.py
再跑本脚本：python3 test_stack_loopback.py [--duration 20]

模型：直道（curvature=0），自车自行车模型；前车 60m 处以 8m/s 匀速。
通过判据：
  1. 收到 /adas/vehicle/actuation_cmd（栈的执行输出回流）
  2. 自车被驱动起步（v > 1 m/s）
  3. 二选一：
     a) ACC 跟车：保持前车后方（gap > 5m）且横向保持（|lat| < 0.8m）
     b) 安全超车：越过前车瞬间已在邻道（|lat| > 2m）——
        栈的 behavior_planner 默认开超车，慢速前车会触发借道超越
  发布速率：odom/steer 50 Hz，lane/objects 20 Hz（与桥节点一致）。
与 bridge_node.publish_frame 的消息组装保持一致（同一份契约的镜像）。
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from adas_msgs.msg import (ActuationCommand, LaneState, SteeringReport,
                           TrackedObject, TrackedObjectArray)
from nav_msgs.msg import Odometry

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
WHEELBASE_M = 2.9
MAX_STEER_RAD = 0.6109      # 35°
LEAD_GAP0_M = 60.0
LEAD_SPEED_MPS = 8.0


def populate_loopback_objects(message, lead_x, actor_count):
    """Populate an ID-sorted N-target array matching bridge topic semantics."""
    message.primary_lead_id = -1
    for index in range(actor_count):
        obj = TrackedObject()
        obj.id = index + 1
        obj.classification = TrackedObject.CLASS_CAR
        obj.pose.pose.position.x = lead_x + 30.0 * index
        obj.pose.pose.position.y = 0.0 if index == 0 else (
            3.5 if index % 2 else -3.5)
        obj.pose.pose.orientation.w = 1.0
        obj.twist.twist.linear.x = LEAD_SPEED_MPS + 0.25 * index
        obj.dimensions.x, obj.dimensions.y, obj.dimensions.z = 4.5, 1.8, 1.5
        message.objects.append(obj)
    return message


def test_n_target_topic_structure_is_sorted_and_primary_is_unset():
    message = populate_loopback_objects(TrackedObjectArray(), 60.0, 20)

    assert len(message.objects) == 20
    assert [obj.id for obj in message.objects] == list(range(1, 21))
    assert message.primary_lead_id == -1


class LoopbackBridge(Node):
    """直道运动学假 CARLA：发感知四话题，收 actuation_cmd 推进模型。"""

    def __init__(self, actor_count=1):
        super().__init__('carla_bridge_loopback')
        if actor_count < 1:
            raise ValueError('actor_count must be positive')
        self.actor_count = actor_count
        qos = qos_profile_sensor_data
        self.pub_odom = self.create_publisher(
            Odometry, '/adas/localization/kinematic_state', qos)
        self.pub_lane = self.create_publisher(
            LaneState, '/adas/perception/lane_state', qos)
        self.pub_steer = self.create_publisher(
            SteeringReport, '/adas/vehicle/steering_report', qos)
        self.pub_objects = self.create_publisher(
            TrackedObjectArray, '/adas/perception/objects_raw', qos)
        self.create_subscription(
            ActuationCommand, '/adas/vehicle/actuation_cmd',
            self._actuation_cb, 1)

        # 自车状态（直道沿 +x，车道中心 y=0）
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.steer_rad = 0.0
        self.lead_x = LEAD_GAP0_M
        self.n_act = 0
        self.act = {'throttle': 0.0, 'brake': 0.0, 'steer': 0.0}
        self.max_abs_lat = 0.0
        self.lat_at_pass = None    # 越过前车瞬间的横向偏移（超车安全性）
        self.prev_gap = LEAD_GAP0_M

        # 50 Hz: odom + steer（控制环必需）
        self.timer = self.create_timer(DT, self._step)
        # 20 Hz: lane + objects（感知/车道行业基准）
        self.timer_lane = self.create_timer(0.05, self._publish_lane)
        self.timer_objects = self.create_timer(0.05, self._publish_objects)

    def _actuation_cb(self, msg):
        self.n_act += 1
        self.act = {'throttle': float(msg.throttle),
                    'brake': float(msg.brake),
                    'steer': float(msg.steer)}

    def _step(self):
        # 执行量 → 自行车模型（粗略纵向：throttle 3m/s²、brake -8m/s²、阻力）
        a = 3.0 * self.act['throttle'] - 8.0 * self.act['brake'] - 0.05 * self.v
        self.v = max(0.0, self.v + a * DT)
        self.steer_rad = self.act['steer'] * MAX_STEER_RAD
        self.yaw += self.v / WHEELBASE_M * math.tan(self.steer_rad) * DT
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT
        self.lead_x += LEAD_SPEED_MPS * DT
        self.max_abs_lat = max(self.max_abs_lat, abs(self.y))
        gap = self.lead_x - self.x
        if self.lat_at_pass is None and self.prev_gap > 0.0 >= gap:
            self.lat_at_pass = self.y
        self.prev_gap = gap

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = (
            self.v / WHEELBASE_M * math.tan(self.steer_rad))
        self.pub_odom.publish(odom)

        steer = SteeringReport()
        steer.header.stamp = now
        steer.steering_tire_angle_rad = float(self.steer_rad)
        self.pub_steer.publish(steer)

    def _publish_lane(self):
        lane = LaneState()
        lane.header.stamp = self.get_clock().now().to_msg()
        lane.header.frame_id = 'base_link'
        lane.valid = True
        lane.lateral_offset = self.y          # 直道：车道中心 y=0，左正
        lane.heading_error = self.yaw
        lane.curvature = 0.0
        lane.lane_width = 3.5
        self.pub_lane.publish(lane)

    def _publish_objects(self):
        objects = TrackedObjectArray()
        objects.header.stamp = self.get_clock().now().to_msg()
        objects.header.frame_id = 'odom'
        populate_loopback_objects(objects, self.lead_x, self.actor_count)
        self.pub_objects.publish(objects)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--actor-count', type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = LoopbackBridge(actor_count=args.actor_count)
    t_end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    gap = node.lead_x - node.x
    print('--- loopback 结果 ---')
    print('actuation_cmd 收到帧数 : %d' % node.n_act)
    print('末速 v                : %.2f m/s' % node.v)
    print('前车间距 gap          : %.1f m' % gap)
    print('最大横向偏移          : %.3f m' % node.max_abs_lat)
    print('最后执行量            : thr=%.2f brk=%.2f steer=%+.3f'
          % (node.act['throttle'], node.act['brake'], node.act['steer']))
    if node.lat_at_pass is not None:
        print('越过前车瞬间横向偏移  : %+.2f m（>2m = 已在邻道，安全超车）'
              % node.lat_at_pass)

    flowing = node.n_act > 50 and node.v > 1.0
    acc_follow = gap > 5.0 and node.max_abs_lat < 0.8
    safe_overtake = (node.lat_at_pass is not None
                     and abs(node.lat_at_pass) > 2.0)
    ok = flowing and (acc_follow or safe_overtake)
    print('判定：%s（%s）' % (
        'PASS' if ok else 'FAIL',
        'ACC 跟车' if acc_follow else ('安全超车' if safe_overtake else '—')))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
