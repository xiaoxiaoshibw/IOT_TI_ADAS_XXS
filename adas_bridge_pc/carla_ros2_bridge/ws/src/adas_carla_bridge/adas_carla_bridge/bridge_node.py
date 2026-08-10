#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CARLA ↔ ADAS SoC 栈 ROS2 桥节点（IOT_TI HIL 闭环 PC 端）。

运行位置：装有 CARLA 的 Ubuntu 24.04 + ROS2 Jazzy（Python 3.12）。
Orin Nano 上的 ADAS0.0.2 C++ 栈（~/adas/adas_soc_ws，ROS2 Humble）跨网 DDS
订阅本节点发布的感知话题——两端 ROS_DOMAIN_ID 必须一致。

话题契约（替代 adas_sim_vehicle，与 sim_vehicle_node.cpp 字节级对齐）：
  发布（SensorDataQoS，真值→感知）：
    /adas/localization/kinematic_state   nav_msgs/Odometry         50 Hz
    /adas/perception/lane_state          adas_msgs/LaneState       20 Hz
    /adas/vehicle/steering_report        adas_msgs/SteeringReport  50 Hz
    /adas/perception/objects_raw         adas_msgs/TrackedObjectArray  20 Hz
  发布速率按行业基准：定位/车辆状态 50 Hz，感知/车道 20 Hz。
  订阅（执行回路）：
    /adas/vehicle/actuation_cmd          adas_msgs/ActuationCommand

执行回路的两种模式（--control-source）：
  ros2  订阅 /adas/vehicle/actuation_cmd 直接驱动 CARLA（调通/桌面联调，默认）
  can   SocketCAN 或 CANalyst-II 收 MCU 0x201 最终控制帧 → CARLA

线程模型：主线程跑 CARLA 同步步进环（world.tick 阻塞），executor 后台
spin 处理订阅回调；执行量带锁存取，stale 超时安全制动兜底。
"""

import argparse
import csv
import json
import math
import os
import threading
import time
from datetime import datetime


def _import_carla():
    try:
        import carla
        return carla
    except ImportError:
        raise SystemExit(
            "无法 import carla。请先在与 rclpy 同一个 Python 环境里安装 CARLA 客户端：\n"
            "  pip install carla==0.9.16\n"
            "或使用 CARLA Linux 发行包内 PythonAPI/carla/dist 的 cp312 wheel。"
        )


import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from adas_msgs.msg import (ActuationCommand, LaneConnection, LaneGraph,
                           LaneState, MapLane, SteeringReport, TrackedObject,
                           TrackedObjectArray)
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from adas_carla_bridge.carla_world import CarlaWorld
from adas_carla_bridge.can_protocol import CanalystReceiver, SocketCanReceiver
from adas_carla_bridge.map_export import export_lane_graph
from adas_carla_bridge.scenarios import ORDER, SCENARIOS

TOPIC_ODOM = '/adas/localization/kinematic_state'
TOPIC_LANE = '/adas/perception/lane_state'
TOPIC_STEER = '/adas/vehicle/steering_report'
TOPIC_OBJECTS = '/adas/perception/objects_raw'
TOPIC_ACTUATION = '/adas/vehicle/actuation_cmd'
TOPIC_CPP_CAN_ACTUATION = '/adas/pc/mcu_actuation'
TOPIC_MAP = '/adas/map/lane_graph'
TOPIC_FAULT_COMMAND = '/adas/_debug/fault_inject_cmd'
TOPIC_FAULT_ACK = '/adas/_debug/fault_inject_ack'

# 地图一次性发布：小队列 + transient_local，晚订阅的 global_planner 也能收到最后一帧。
# reliability 显式写死 RELIABLE：订阅端是 reliable，若这里依赖 RMW 默认值、一旦被
# 解析成 BEST_EFFORT，reliable 订阅端会静默收不到地图且无任何报错。
MAP_QOS = QoSProfile(depth=1)
MAP_QOS.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
MAP_QOS.reliability = QoSReliabilityPolicy.RELIABLE

# 跨 ROS 2 Jazzy/Humble 的 HIL 真值链路使用 RELIABLE 发布。可靠发布端仍可
# 匹配 SensorDataQoS(BEST_EFFORT) 订阅端，同时也兼容控制栈中要求 RELIABLE
# 的订阅端；反向使用 BEST_EFFORT 发布会让后者完全收不到里程计。
SENSOR_QOS = QoSProfile(depth=5)
SENSOR_QOS.durability = QoSDurabilityPolicy.VOLATILE
SENSOR_QOS.reliability = QoSReliabilityPolicy.RELIABLE

# 执行链路丢失时的安全制动兜底（归一化制动量）
FAILSAFE_BRAKE = 1.0
VALID_RECOVERY_FRAMES = 3


def validate_actuation_values(throttle, brake, steer):
    """校验跨机执行量；返回 (合法, 原因)。非法帧不得刷新活性时间。"""
    values = (float(throttle), float(brake), float(steer))
    if not all(math.isfinite(value) for value in values):
        return False, 'non_finite'
    if not 0.0 <= values[0] <= 1.0:
        return False, 'throttle_out_of_range'
    if not 0.0 <= values[1] <= 1.0:
        return False, 'brake_out_of_range'
    if not -1.0 <= values[2] <= 1.0:
        return False, 'steer_out_of_range'
    if values[0] > 1e-3 and values[1] > 1e-3:
        return False, 'throttle_brake_conflict'
    return True, 'ok'


class CarlaBridgeNode(Node):
    """发布 CARLA 真值感知话题；订阅执行量，供步进环带锁读取。"""

    def __init__(self, args):
        super().__init__('carla_bridge')
        self.args = args
        self._lock = threading.Lock()
        self._act = {'throttle': 0.0, 'brake': 0.0, 'steer': 0.0, 'rx_t': 0.0}
        self._stale_timeout_s = float(args.stale_timeout_s)
        if not math.isfinite(self._stale_timeout_s) or self._stale_timeout_s <= 0.0:
            raise SystemExit('--stale-timeout-s 必须是有限正数')
        self._invalid_latched = False
        self._invalid_count = 0
        self._valid_recovery_frames = 0
        self._can_receiver = None
        self._fault_sequence = 0

        self.pub_odom = self.create_publisher(Odometry, TOPIC_ODOM, SENSOR_QOS)
        self.pub_lane = self.create_publisher(LaneState, TOPIC_LANE, SENSOR_QOS)
        self.pub_steer = self.create_publisher(SteeringReport, TOPIC_STEER,
                                               SENSOR_QOS)
        self.pub_objects = self.create_publisher(TrackedObjectArray,
                                                 TOPIC_OBJECTS, SENSOR_QOS)
        self.pub_map = self.create_publisher(LaneGraph, TOPIC_MAP, MAP_QOS)
        self.pub_fault_ack = self.create_publisher(String, TOPIC_FAULT_ACK, 10)
        self.create_subscription(String, TOPIC_FAULT_COMMAND,
                                 self._fault_inject_cb, 10)

        # 最新帧缓存（主线程写入，20 Hz 定时器读取）
        self._latest_frame = None
        self._frame_lock = threading.Lock()

        # 感知/车道按 20 Hz 发布（行业基准）
        SLOW_RATE = 0.05
        self._timer_lane = self.create_timer(SLOW_RATE, self._publish_lane)
        self._timer_objects = self.create_timer(SLOW_RATE, self._publish_objects)

        if args.control_source in ('ros2', 'can_cpp'):
            topic = (args.can_cpp_topic if args.control_source == 'can_cpp'
                     else TOPIC_ACTUATION)
            self.create_subscription(ActuationCommand, topic,
                                     self._actuation_cb, 1)
        else:
            try:
                if args.can_transport == 'canalystii':
                    self._can_receiver = CanalystReceiver(
                        device=args.can_device_index,
                        channel=args.can_channel,
                        bitrate=args.can_bitrate,
                        feedback_timeout_s=args.can_feedback_timeout_s)
                else:
                    self._can_receiver = SocketCanReceiver(
                        args.can_interface,
                        feedback_timeout_s=args.can_feedback_timeout_s)
            except (OSError, RuntimeError, ValueError) as error:
                raise SystemExit(
                    '无法打开 %s CAN 接收端：%s'
                    % (args.can_transport, error)) from error

        self.get_logger().info(
            'CARLA 桥就绪：scenario=%s control=%s domain=%s'
            % (args.scenario, args.control_source,
               os.environ.get('ROS_DOMAIN_ID', '0')))

    def _actuation_cb(self, msg):
        valid, reason = validate_actuation_values(msg.throttle, msg.brake, msg.steer)
        with self._lock:
            if not valid:
                self._invalid_latched = True
                self._valid_recovery_frames = 0
                self._invalid_count += 1
                invalid_count = self._invalid_count
            else:
                self._act['throttle'] = float(msg.throttle)
                self._act['brake'] = float(msg.brake)
                self._act['steer'] = float(msg.steer)
                self._act['rx_t'] = time.monotonic()
                if self._invalid_latched:
                    self._valid_recovery_frames += 1
                    if self._valid_recovery_frames >= VALID_RECOVERY_FRAMES:
                        self._invalid_latched = False
                invalid_count = self._invalid_count
        if not valid and (invalid_count == 1 or invalid_count % 100 == 0):
            self.get_logger().error(
                '拒绝非法执行帧：%s（累计 %d）' % (reason, invalid_count))

    def _fault_inject_cb(self, msg):
        request_id = ''
        command = -1
        parameter = 0
        accepted = False
        detail = ''
        try:
            payload = json.loads(msg.data)
            request_id = str(payload.get('request_id', ''))
            command = int(payload.get('cmd', -1))
            parameter = int(payload.get('param', 0))
            if not request_id:
                raise ValueError('missing request_id')
            if self._can_receiver is None:
                raise RuntimeError('CAN transport is not active for this bridge')
            self._fault_sequence = (self._fault_sequence + 1) & 0xFF
            self._can_receiver.send_fault_injection(
                command, parameter, self._fault_sequence)
            accepted = True
            detail = '0x301 transport send completed'
        except (ValueError, TypeError, RuntimeError, OSError) as error:
            detail = str(error)
        ack = String()
        ack.data = json.dumps({
            'request_id': request_id, 'cmd': command,
            'accepted': accepted, 'detail': detail,
            'source': 'adas_carla_bridge'}, separators=(',', ':'))
        self.pub_fault_ack.publish(ack)

    # ── 地图一次性发布（M6 第 1 步：CARLA→lane_graph）──

    def publish_map(self, graph_dict):
        """map_export.export_lane_graph 的字典 → LaneGraph 消息，发布一次。

        transient_local QoS 保证晚启动的 global_planner 仍能收到最后一帧。"""
        msg = LaneGraph()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.map_id = graph_dict['map_id']
        msg.map_hash = graph_dict['map_hash']
        n_conn = 0
        for lane_dict in graph_dict['lanes']:
            lane = MapLane()
            lane.id = int(lane_dict['id'])
            lane.speed_limit_mps = float(lane_dict['speed_limit_mps'])
            lane.junction = bool(lane_dict['junction'])
            for x, y, yaw in lane_dict['centerline']:
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.orientation.z = math.sin(yaw / 2.0)
                pose.orientation.w = math.cos(yaw / 2.0)
                lane.centerline.append(pose)
            for edge in lane_dict['outgoing']:
                conn = LaneConnection()
                conn.to_lane_id = int(edge['to_lane_id'])
                conn.maneuver = int(edge['maneuver'])
                conn.extra_cost_m = float(edge['extra_cost_m'])
                lane.outgoing.append(conn)
                n_conn += 1
            msg.lanes.append(lane)
        self.pub_map.publish(msg)
        self.get_logger().info(
            '发布地图 %s：%d 车道 / %d 连接（map_hash=%s…）'
            % (msg.map_id, len(msg.lanes), n_conn, msg.map_hash[:8]))

    # ── 步进环线程调用 ──

    def get_actuation(self):
        if self._can_receiver is not None:
            return self._can_receiver.current()
        with self._lock:
            act = dict(self._act)
            invalid_latched = self._invalid_latched
            invalid_count = self._invalid_count
        age = time.monotonic() - act['rx_t'] if act['rx_t'] else float('inf')
        if invalid_latched or age > self._stale_timeout_s:
            return {'throttle': 0.0, 'brake': FAILSAFE_BRAKE, 'steer': 0.0,
                    'stale': age > self._stale_timeout_s,
                    'invalid_latched': invalid_latched,
                    'invalid_count': invalid_count, 'age_s': age}
        act['stale'] = False
        act['invalid_latched'] = False
        act['invalid_count'] = invalid_count
        act['age_s'] = age
        return act

    def close(self):
        if self._can_receiver is not None:
            self._can_receiver.close()
            self._can_receiver = None

    def publish_frame(self, frame):
        now = self.get_clock().now().to_msg()
        ego = frame['ego']

        with self._frame_lock:
            self._latest_frame = frame

        # 50 Hz: 里程计 + 转向报告（高动态控制必需）
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = ego['x']
        odom.pose.pose.position.y = ego['y']
        odom.pose.pose.orientation.z = math.sin(ego['yaw'] / 2.0)
        odom.pose.pose.orientation.w = math.cos(ego['yaw'] / 2.0)
        odom.twist.twist.linear.x = ego['v']
        odom.twist.twist.angular.z = ego['yaw_rate']
        self.pub_odom.publish(odom)

        steer = SteeringReport()
        steer.header.stamp = now
        steer.steering_tire_angle_rad = float(ego['steer_rad'])
        self.pub_steer.publish(steer)

        # 20 Hz: lane_state + objects_raw 由 _publish_lane / _publish_objects 定时器发布

    def _publish_lane(self):
        with self._frame_lock:
            if self._latest_frame is None:
                return
            ls = self._latest_frame['lane']
        lane = LaneState()
        lane.header.stamp = self.get_clock().now().to_msg()
        lane.header.frame_id = 'base_link'
        lane.valid = bool(ls['valid'])
        lane.lateral_offset = float(ls['lateral_offset'])
        lane.heading_error = float(ls['heading_error'])
        lane.curvature = float(ls['curvature'])
        lane.lane_width = float(ls['lane_width'])
        self.pub_lane.publish(lane)

    def _publish_objects(self):
        with self._frame_lock:
            if self._latest_frame is None:
                return
            objs = self._latest_frame['objects']
        objects = TrackedObjectArray()
        objects.header.stamp = self.get_clock().now().to_msg()
        objects.header.frame_id = 'odom'
        objects.primary_lead_id = -1
        for obj in objs:
            o = TrackedObject()
            o.id = int(obj['id'])
            o.classification = int(obj['cls'])
            o.pose.pose.position.x = obj['x']
            o.pose.pose.position.y = obj['y']
            o.pose.pose.orientation.z = math.sin(obj['yaw'] / 2.0)
            o.pose.pose.orientation.w = math.cos(obj['yaw'] / 2.0)
            o.twist.twist.linear.x = obj['v']
            o.dimensions.x, o.dimensions.y, o.dimensions.z = obj['dims']
            objects.objects.append(o)
        self.pub_objects.publish(objects)


def carla_loop(node, world, args, scenario):
    """主线程步进环：tick → sense → publish → 读执行量 → apply → 记 CSV。"""
    duration = float(args.duration if args.duration is not None
                     else scenario.get('duration', 0.0))
    log_dir = os.path.abspath(args.log_dir)
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, 'carla_%s_%s.csv' % (
        args.scenario, datetime.now().strftime('%Y%m%d_%H%M%S')))
    print('CSV log: %s' % log_path, flush=True)

    seq = 0
    last_print = 0.0
    start_wall = time.monotonic()
    with open(log_path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow([
            't', 'seq', 'ego_v', 'lat_offset', 'heading_err', 'curvature',
            'n_objects', 'throttle', 'brake', 'steer', 'stale',
            'invalid_latched', 'invalid_count', 'act_age_s',
        ])
        while rclpy.ok():
            sim_t = world.tick()
            if duration > 0.0 and sim_t >= duration:
                break

            frame = world.sense()
            node.publish_frame(frame)
            world.drive_actors(sim_t)

            act = node.get_actuation()
            throttle, brake, steer = world.apply_ego(
                act['throttle'], act['brake'], act['steer'])
            world.update_spectator()

            writer.writerow([
                '%.3f' % sim_t, seq,
                '%.3f' % frame['ego']['v'],
                '%.3f' % frame['lane']['lateral_offset'],
                '%.4f' % frame['lane']['heading_error'],
                '%.5f' % frame['lane']['curvature'],
                len(frame['objects']),
                '%.3f' % throttle, '%.3f' % brake, '%.4f' % steer,
                int(act['stale']),
                int(act['invalid_latched']), act['invalid_count'],
                '%.3f' % act['age_s'] if act['age_s'] != float('inf') else 'inf',
            ])

            if sim_t - last_print >= 1.0:
                fh.flush()
                print(
                    't=%6.1f v=%5.2f lat=%+.2f objs=%d thr=%.2f brk=%.2f '
                    'steer=%+.3f%s'
                    % (sim_t, frame['ego']['v'],
                       frame['lane']['lateral_offset'],
                       len(frame['objects']), throttle, brake, steer,
                        ('  [INVALID→failsafe]' if act['invalid_latched'] else
                         ('  [STALE→failsafe]' if act['stale'] else ''))),
                    flush=True)
                last_print = sim_t

            seq += 1
            if args.realtime:
                target = start_wall + sim_t
                sleep_s = target - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(min(sleep_s, 0.05))


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description='CARLA ↔ ADAS SoC 栈 ROS2 桥（IOT_TI HIL 闭环 PC 端）')
    parser.add_argument('--scenario', default='acc', choices=ORDER)
    parser.add_argument('--control-source', choices=['ros2', 'can', 'can_cpp'],
                        default='ros2',
                        help=('ros2=订阅 actuation_cmd；can=Python SocketCAN；'
                              'can_cpp=订阅 C++ SocketCAN 回控节点'))
    parser.add_argument('--stale-timeout-s', type=float, default=0.5,
                        help='执行量超过此时长未更新 → 安全制动兜底')
    parser.add_argument('--can-interface', default='can0',
                        help='can-transport=socketcan 时使用的接口')
    parser.add_argument('--can-transport', choices=['socketcan', 'canalystii'],
                        default='socketcan',
                        help='control-source=can 时使用的 CAN 接收后端')
    parser.add_argument('--can-device-index', type=int, default=0,
                        help='CANalyst-II USB 设备序号（从 0 开始）')
    parser.add_argument('--can-channel', type=int, choices=[0, 1], default=1,
                        help='CANalyst-II 通道序号（0=CAN1，1=CAN2）')
    parser.add_argument('--can-bitrate', type=int, default=500000,
                        help='CANalyst-II 波特率')
    parser.add_argument('--can-feedback-timeout-s', type=float, default=0.1,
                        help='MCU 反馈超时后进入全制动兜底的时间')
    parser.add_argument('--can-cpp-topic', default=TOPIC_CPP_CAN_ACTUATION,
                        help=('control-source=can_cpp 时的 ActuationCommand 话题；'
                              '可直接使用 Orin 网关发布的 MCU 回控'))
    parser.add_argument('--carla-host', default='127.0.0.1')
    parser.add_argument('--carla-port', type=int, default=2000)
    parser.add_argument('--town', default='Town04')
    parser.add_argument('--fixed-dt', type=float, default=0.02,
                        help='CARLA 同步步长（0.02=50Hz，与 sim_vehicle 一致）')
    parser.add_argument('--spawn-index', type=int, default=None)
    parser.add_argument('--duration', type=float, default=None)
    parser.add_argument('--log-dir', default='logs')
    parser.add_argument('--no-map', action='store_true',
                        help='不导出/发布 /adas/map/lane_graph（关闭导航链路）')
    parser.add_argument('--map-sample-m', type=float, default=2.0,
                        help='车道中心线采样间距 [m]（越小越精细、消息越大）')
    parser.add_argument('--no-rendering', action='store_true')
    parser.add_argument('--no-realtime', dest='realtime', action='store_false')
    parser.set_defaults(realtime=True)
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    carla = _import_carla()
    scenario = SCENARIOS[args.scenario]

    print('场景：%s' % scenario['name'], flush=True)
    for line in scenario.get('notes', []):
        print('  · %s' % line, flush=True)

    rclpy.init()
    world = CarlaWorld(
        carla, args.carla_host, args.carla_port, scenario,
        town=args.town, fixed_dt=args.fixed_dt,
        no_rendering=args.no_rendering, spawn_index=args.spawn_index)
    node = CarlaBridgeNode(args)

    if not args.no_map:
        graph_dict = export_lane_graph(
            world.map, sample_distance_m=args.map_sample_m)
        node.publish_map(graph_dict)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name='ros-spin',
                                   daemon=True)
    spin_thread.start()

    try:
        carla_loop(node, world, args, scenario)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            world.close()
        finally:
            executor.shutdown()
            node.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
