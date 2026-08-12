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
import re
import threading
import time
import uuid
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


_UUID_V4_RE = re.compile(
    r'^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$')


def is_canonical_uuid_v4(value):
    """P0.C: 严格校验 UUID v4（小写、version 4、variant 8/9/a/b）。"""
    if not isinstance(value, str):
        return False
    if not _UUID_V4_RE.match(value):
        return False
    try:
        parsed = uuid.UUID(value)
    except (ValueError, TypeError):
        return False
    return parsed.version == 4 and str(parsed) == value


import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster

from adas_msgs.msg import (ActuationCommand, AebStatus, BehaviorState,
                           GlobalRoute, LaneConnection, LaneGraph, LaneState,
                           MapLane, NavigationStatus, SafetyStatus,
                           SteeringReport, TrackedObject, TrackedObjectArray)
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from adas_carla_bridge.carla_world import CarlaWorld
from adas_carla_bridge.can_protocol import CanalystReceiver, SocketCanReceiver
from adas_carla_bridge.map_export import export_lane_graph
from adas_carla_bridge.scenario_loader import (
    ScenarioLoadError,
    known_scenario_ids,
    load_scenario,
)

TOPIC_ODOM = '/adas/localization/kinematic_state'
TOPIC_LANE = '/adas/perception/lane_state'
TOPIC_STEER = '/adas/vehicle/steering_report'
TOPIC_OBJECTS = '/adas/perception/objects_raw'
TOPIC_ACTUATION = '/adas/vehicle/actuation_cmd'
TOPIC_CPP_CAN_ACTUATION = '/adas/pc/mcu_actuation'
TOPIC_MAP = '/adas/map/lane_graph'
TOPIC_FAULT_COMMAND = '/adas/_debug/fault_inject_cmd'
TOPIC_FAULT_ACK = '/adas/_debug/fault_inject_ack'
TOPIC_ROUTE = '/adas/planning/global_route'
TOPIC_NAV_ROUTE = '/adas/navigation/global_route'
TOPIC_NAV_STATUS = '/adas/navigation/status'
TOPIC_BEHAVIOR = '/adas/planning/behavior'
TOPIC_AEB_STATUS = '/adas/control/aeb/status'
TOPIC_SAFETY_STATUS = '/adas/system/safety_status'
TOPIC_CARLA_VISUALIZATION = '/adas/ui/carla_visualization_cmd'

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

# P0.D: 视觉/状态 fresh 性默认阈值。route/goal/behavior/AEB/safety 任一
# 字段超过此时长未更新，即认为上游断流，桥实例必须把对应 overlay 清空
# 或把状态值替换为 STALE/UNKNOWN，禁止继续显示旧值。
# 4 × stale_timeout_s 是经验值（控制回路通常 10–50 Hz，4 倍默认 0.5 s
# 下界为 2 s，足以区分短暂阻塞与真实断流）。
def _default_freshness_timeout_s(stale_timeout_s: float) -> float:
    return max(1.0, 4.0 * float(stale_timeout_s))


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
        # P0.C/P0.3: 本次桥实例持有的 run_id。GUI/Orchestrator 通过 --run-id
        # 注入；只有显式 --auto-run-id（仅用于独立调试/测试）才允许自动
        # 生成 UUID v4。其他场景缺省/非法值必须直接 fail-closed，避免两端
        # 各自生成 ID 导致跨进程无法握手。
        raw_run_id = str(getattr(args, 'run_id', '') or '').strip()
        auto_run_id = bool(getattr(args, 'auto_run_id', False))
        if raw_run_id:
            if not is_canonical_uuid_v4(raw_run_id):
                raise SystemExit(
                    '--run-id 必须是规范 UUID v4（小写、version 4、variant 8/9/a/b）'
                    ';若希望启动时自动生成，请显式传 --auto-run-id')
            self._current_run_id = raw_run_id
        elif auto_run_id:
            self._current_run_id = str(uuid.uuid4())
        else:
            raise SystemExit(
                '--run-id 是必填项（必须是规范 UUID v4）。'
                '独立调试可额外传 --auto-run-id 自动生成，'
                '但生产/GUI/Orchestrator 调用必须由上游注入同一个 UUID v4')
        self._stale_timeout_s = float(args.stale_timeout_s)
        if not math.isfinite(self._stale_timeout_s) or self._stale_timeout_s <= 0.0:
            raise SystemExit('--stale-timeout-s 必须是有限正数')
        self._invalid_latched = False
        self._invalid_count = 0
        self._valid_recovery_frames = 0
        self._can_receiver = None
        self._fault_sequence = 0
        # CARLA 是唯一场景展示端。以下缓存由 ROS 回调更新，由主 CARLA tick
        # 线程读取后交给 CarlaWorld 绘制，避免从 executor 线程调用 CARLA API。
        self._visual_lock = threading.Lock()
        self._freshness_timeout_s = _default_freshness_timeout_s(self._stale_timeout_s)
        self._visual = {
            'route': [], 'route_seen_t': 0.0,
            'nav_state': 0, 'status_seen_t': 0.0,
            'remaining_m': float('nan'),
            'behavior_state': -1, 'behavior_seen_t': 0.0,
            'target_speed_mps': float('nan'),
            'aeb_state': 0, 'aeb_seen_t': 0.0,
            'ttc_s': float('nan'),
            'safety_level': 0, 'safety_seen_t': 0.0,
            'pending_goal': None, 'goal_seen_t': 0.0,
        }

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

        visual_qos = QoSProfile(depth=1)
        visual_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        visual_qos.reliability = QoSReliabilityPolicy.RELIABLE
        # P0.C/P0.3: 路线显示完全由带 run_id 的 GlobalRoute 驱动。
        # /adas/planning/global_route 是 nav_msgs/Path,只供控制栈 (trajectory_planner)
        # 使用,且由 route_adapter_node 唯一发布;bridge 不订阅以避免
        # 旧会话或无 run_id 的 Path 绕回 CARLA overlay。
        self.create_subscription(GlobalRoute, TOPIC_NAV_ROUTE,
                                 self._nav_route_cb, visual_qos)
        self.create_subscription(NavigationStatus, TOPIC_NAV_STATUS,
                                 self._nav_status_cb, visual_qos)
        self.create_subscription(BehaviorState, TOPIC_BEHAVIOR,
                                 self._behavior_cb, 10)
        self.create_subscription(AebStatus, TOPIC_AEB_STATUS,
                                 self._aeb_cb, 10)
        self.create_subscription(SafetyStatus, TOPIC_SAFETY_STATUS,
                                 self._safety_cb, 10)
        self.create_subscription(String, TOPIC_CARLA_VISUALIZATION,
                                 self._visualization_command_cb, visual_qos)

        # 最新帧缓存（主线程写入，20 Hz 定时器读取）
        self._latest_frame = None
        self._frame_lock = threading.Lock()

        # 感知/车道按 20 Hz 发布（行业基准）
        SLOW_RATE = 0.05
        self._timer_lane = self.create_timer(SLOW_RATE, self._publish_lane)
        self._timer_objects = self.create_timer(SLOW_RATE, self._publish_objects)
        # P0.D: 1 Hz 看门狗，巡检 overlay/state 是否 fresh。
        self._timer_freshness = self.create_timer(1.0, self._freshness_watchdog)

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

        # 静态 TF：map -> odom。CARLA 链路里地图世界系即等于里程计原点，
        # 与 SIL 节点行为一致；tf_chain_test 才看得到 map->odom->base_link。
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        static_map_odom = TransformStamped()
        static_map_odom.header.stamp = self.get_clock().now().to_msg()
        static_map_odom.header.frame_id = 'map'
        static_map_odom.child_frame_id = 'odom'
        static_map_odom.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(static_map_odom)

        # 动态 TF：odom -> base_link。每帧 odom 同步广播，保证下游 TF lookup
        # 连续成功且数值有限（tf_chain_test 直接消费）。
        self._dynamic_tf_broadcaster = TransformBroadcaster(self)

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

    def _accept_run_id(self, msg_run_id: str) -> bool:
        """P0.C: 消费端会话过滤。空 run_id 一律拒绝（不含通配语义）。"""
        if not msg_run_id or not self._current_run_id:
            return False
        return msg_run_id == self._current_run_id

    def _nav_route_cb(self, msg):
        # P0.C: 带 run_id 的 GlobalRoute 是当前会话路线；空 run_id 直接丢弃。
        if not self._accept_run_id(getattr(msg, 'run_id', '')):
            return
        # 非 VALID 状态视为清空指令（INVALID/FAILED/CANCELLED/ARRIVED）。
        if int(msg.status) != GlobalRoute.STATUS_VALID or len(msg.points) < 2:
            with self._visual_lock:
                self._visual['route'] = []
                self._visual['route_seen_t'] = 0.0
            return
        points = [(float(p.x), float(p.y))
                  for p in msg.points
                  if math.isfinite(float(p.x)) and math.isfinite(float(p.y))]
        if len(points) > 300:
            endpoint = points[-1]
            stride = int(math.ceil(len(points) / 300.0))
            points = points[::stride]
            if points[-1] != endpoint:
                points.append(endpoint)
        with self._visual_lock:
            self._visual['route'] = points
            self._visual['route_seen_t'] = time.monotonic()

    def _nav_status_cb(self, msg):
        # P0.C: 旧会话残留的 status 不应继续作为当前导航状态。
        if not self._accept_run_id(getattr(msg, 'run_id', '')):
            return
        with self._visual_lock:
            self._visual['nav_state'] = int(msg.state)
            self._visual['remaining_m'] = float(msg.remaining_distance_m)
            self._visual['status_seen_t'] = time.monotonic()
            if int(msg.state) in (NavigationStatus.ARRIVED,
                                  NavigationStatus.FAILED,
                                  NavigationStatus.CANCELED):
                self._visual['route'] = []
                self._visual['pending_goal'] = None
                self._visual['route_seen_t'] = 0.0
                self._visual['goal_seen_t'] = 0.0

    def _behavior_cb(self, msg):
        with self._visual_lock:
            self._visual['behavior_state'] = int(msg.state)
            self._visual['target_speed_mps'] = float(msg.target_speed_mps)
            self._visual['behavior_seen_t'] = time.monotonic()

    def _aeb_cb(self, msg):
        with self._visual_lock:
            self._visual['aeb_state'] = int(msg.state)
            self._visual['ttc_s'] = float(msg.ttc_s)
            self._visual['aeb_seen_t'] = time.monotonic()

    def _safety_cb(self, msg):
        with self._visual_lock:
            self._visual['safety_level'] = int(msg.overall)
            self._visual['safety_seen_t'] = time.monotonic()

    def _visualization_command_cb(self, msg):
        try:
            command = json.loads(msg.data)
            operation = str(command.get('operation', ''))
            if operation == 'goal':
                x = float(command['x'])
                y = float(command['y'])
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError('non-finite CARLA visualization goal')
                goal = (x, y)
            elif operation == 'cancel':
                goal = None
            else:
                raise ValueError('unsupported visualization operation')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning('忽略非法 CARLA 展示命令：%s' % error)
            return
        with self._visual_lock:
            self._visual['pending_goal'] = goal
            self._visual['goal_seen_t'] = time.monotonic() if goal is not None else 0.0
            if goal is None:
                self._visual['route'] = []
                self._visual['route_seen_t'] = 0.0

    def get_visualization_state(self):
        """返回可由 CARLA 主线程安全消费的展示快照。"""
        with self._visual_lock:
            state = dict(self._visual)
            state['route'] = list(self._visual['route'])
            return state

    def _freshness_watchdog(self):
        """P0.D: 巡检各字段 last_seen 时间戳，超时即清空或替换为 STALE/UNKNOWN。"""
        now = time.monotonic()
        timeout = self._freshness_timeout_s
        with self._visual_lock:
            # route / goal：超时清空 overlay，并把对应 seen_t 归零，避免下次
            # 巡检重复处理同一超时事件。
            if self._visual['route'] and now - self._visual['route_seen_t'] > timeout:
                self._visual['route'] = []
                self._visual['route_seen_t'] = 0.0
            if (self._visual['pending_goal'] is not None
                    and self._visual['goal_seen_t'] > 0.0
                    and now - self._visual['goal_seen_t'] > timeout):
                self._visual['pending_goal'] = None
                self._visual['goal_seen_t'] = 0.0
            # navigation status：超时置为 IDLE + NaN，并把 status_seen_t 归零。
            if (self._visual['status_seen_t'] > 0.0
                    and now - self._visual['status_seen_t'] > timeout):
                self._visual['nav_state'] = NavigationStatus.IDLE
                self._visual['remaining_m'] = float('nan')
                self._visual['status_seen_t'] = 0.0
            # 状态字段：超时置为 STALE/UNKNOWN，避免继续显示陈旧值。
            # 注意：seen_t 初始化为 0.0，必须显式 > 0.0 才视作"已观察过"。
            if (self._visual['behavior_seen_t'] > 0.0
                    and now - self._visual['behavior_seen_t'] > timeout):
                self._visual['behavior_state'] = -1
                self._visual['target_speed_mps'] = float('nan')
                self._visual['behavior_seen_t'] = 0.0
            if (self._visual['aeb_seen_t'] > 0.0
                    and now - self._visual['aeb_seen_t'] > timeout):
                self._visual['aeb_state'] = 0
                self._visual['ttc_s'] = float('nan')
                self._visual['aeb_seen_t'] = 0.0
            if (self._visual['safety_seen_t'] > 0.0
                    and now - self._visual['safety_seen_t'] > timeout):
                self._visual['safety_level'] = 0
                self._visual['safety_seen_t'] = 0.0

    def _fault_inject_cb(self, msg):
        request_id = ''
        command = -1
        parameter = 0
        accepted = False
        detail = ''
        response = None
        timed_out = False
        try:
            payload = json.loads(msg.data)
            request_id = str(payload.get('request_id', ''))
            command = int(payload.get('cmd', -1))
            parameter = int(payload.get('param', 0))
            parsed_id = uuid.UUID(request_id)
            if parsed_id.version != 4 or str(parsed_id) != request_id.lower():
                raise ValueError('request_id must be canonical UUID v4')
            if self._can_receiver is None:
                raise RuntimeError('CAN transport is not active for this bridge')
            self._fault_sequence = (self._fault_sequence + 1) & 0xFF
            response = self._can_receiver.send_fault_injection(
                command, parameter, self._fault_sequence)
            accepted = True
            detail = ('MCU 0x302 ack: state=%d fault_level=%d seq=%d'
                      % (response['system_state'], response['fault_level'],
                         response['sequence']))
        except (ValueError, TypeError, RuntimeError, OSError) as error:
            detail = str(error)
            timed_out = isinstance(error, TimeoutError)
        ack = String()
        ack.data = json.dumps({
            'request_id': request_id, 'cmd': command,
            'accepted': accepted, 'detail': detail,
            'timed_out': timed_out,
            'can_id': '0x302' if response is not None else '',
            'dlc': 8 if response is not None else 0,
            'crc_valid': response is not None,
            'param': parameter,
            'source': 'adas_carla_bridge'}, separators=(',', ':'))
        self.pub_fault_ack.publish(ack)

    # ── 地图一次性发布（M6 第 1 步：CARLA→lane_graph）──

    def publish_map(self, graph_dict):
        """map_export.export_lane_graph 的字典 → LaneGraph 消息，发布一次。

        transient_local QoS 保证晚启动的 global_planner 仍能收到最后一帧。"""
        msg = LaneGraph()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # P0.C: producer 端把本桥的 run_id 写入 LaneGraph，作为后续消费端的会话基准。
        msg.run_id = self._current_run_id
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
        # P0.D: 幂等 stop cleanup。任何字段都允许重入且不得报错。
        if self._can_receiver is not None:
            try:
                self._can_receiver.close()
            except Exception:  # noqa: BLE001 — 关闭期最严格"清干净"原则
                pass
            self._can_receiver = None
        # 取消 freshness watchdog timer；ROS destroy_node 也会停，但显式销毁更稳。
        timer = getattr(self, '_timer_freshness', None)
        if timer is not None:
            try:
                timer.cancel()
            except Exception:  # noqa: BLE001
                pass
        # 清空 overlay/state 缓存；下一次启动从零开始，杜绝旧值残留。
        with self._visual_lock:
            for key in ('route', 'pending_goal'):
                self._visual[key] = [] if key == 'route' else None
            for key in ('nav_state', 'behavior_state', 'aeb_state', 'safety_level'):
                self._visual[key] = 0
            self._visual['behavior_state'] = -1
            for key in ('remaining_m', 'target_speed_mps', 'ttc_s'):
                self._visual[key] = float('nan')
            for key in ('route_seen_t', 'status_seen_t', 'behavior_seen_t',
                        'aeb_seen_t', 'safety_seen_t', 'goal_seen_t'):
                self._visual[key] = 0.0

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

        # 动态 odom -> base_link：与同帧 Odometry 共享时间戳/位姿/姿态，
        # 四元数来自 Odometry orientation（已归一化），保证 tf_chain_test
        # 在 /tf 上能连续查到 map -> odom -> base_link 全链。
        odom_base = TransformStamped()
        odom_base.header.stamp = now
        odom_base.header.frame_id = 'odom'
        odom_base.child_frame_id = 'base_link'
        odom_base.transform.translation.x = float(ego['x'])
        odom_base.transform.translation.y = float(ego['y'])
        odom_base.transform.translation.z = 0.0
        odom_base.transform.rotation = odom.pose.pose.orientation
        # 四元数必须有限；ego yaw 来自 CARLA，是 math.sin/cos 配对构造的，
        # 数值上必然有限；这里仍做一次防御，避免上游 NaN/Inf 污染 tf lookup。
        if not all(math.isfinite(float(v)) for v in (
                odom_base.transform.rotation.x,
                odom_base.transform.rotation.y,
                odom_base.transform.rotation.z,
                odom_base.transform.rotation.w)):
            odom_base.transform.rotation.x = 0.0
            odom_base.transform.rotation.y = 0.0
            odom_base.transform.rotation.z = 0.0
            odom_base.transform.rotation.w = 1.0
        self._dynamic_tf_broadcaster.sendTransform(odom_base)

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
                     else scenario.get('duration_s', 0.0))
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
            world.draw_adas_visualization(
                node.get_visualization_state(), frame, act, sim_t)

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
    parser.add_argument('--scenario', default='acc',
                        help=('catalog scenario ID (known: %s)'
                              % ','.join(known_scenario_ids())))
    parser.add_argument('--scenario-file', default='',
                        help='schema-v1 JSON file; takes priority over --scenario')
    parser.add_argument('--seed', type=int, default=None,
                        help='deterministic seed override; 0 means no perturbation')
    parser.add_argument('--expected-actor-count', type=int, default=None,
                        help='fail startup unless exactly this many actors spawn')
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
    parser.add_argument('--run-id', default='',
                        help='P0.C 会话 run_id；必须是规范 UUID v4（小写）。'
                             ' 缺省或非法值在不传 --auto-run-id 时会 fail-closed，'
                             '确保 GUI/Orchestrator 必须注入同一个 UUID v4。')
    parser.add_argument('--auto-run-id', action='store_true',
                        help='仅用于独立调试/单元测试：缺省 --run-id 时自动生成 UUID v4。'
                             '生产 / GUI / Orchestrator 调用必须依赖 --run-id 显式注入。')
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
    try:
        scenario = load_scenario(
            scenario_id=args.scenario, scenario_file=args.scenario_file,
            seed=args.seed)
    except ScenarioLoadError as error:
        print('场景加载失败：%s' % error, flush=True)
        return 2
    if args.expected_actor_count is not None and args.expected_actor_count < 0:
        print('--expected-actor-count 必须非负', flush=True)
        return 2
    expected_actor_count = (
        len(scenario['actors']) if args.expected_actor_count is None
        else args.expected_actor_count)
    args.scenario = scenario['id']

    carla = _import_carla()

    print('场景：%s' % scenario['name'], flush=True)
    for line in scenario.get('notes', []):
        print('  · %s' % line, flush=True)
    print('  schema=%d id=%s seed=%d expected_actors=%d source=%s' % (
        scenario['schema_version'], scenario['id'], scenario['seed'],
        expected_actor_count, scenario.get('_source_file') or 'legacy'),
        flush=True)

    rclpy.init()
    world = None
    node = None
    executor = None
    exit_code = 0
    try:
        world = CarlaWorld(
            carla, args.carla_host, args.carla_port, scenario,
            town=args.town or scenario['town'], fixed_dt=args.fixed_dt,
            no_rendering=args.no_rendering, spawn_index=args.spawn_index)
        if world.scripted_actor_count != expected_actor_count:
            raise RuntimeError('actor count mismatch: spawned=%d expected=%d' % (
                world.scripted_actor_count, expected_actor_count))
        print('  spawned_actors=%d' % world.scripted_actor_count, flush=True)

        node = CarlaBridgeNode(args)
        if not args.no_map:
            graph_dict = export_lane_graph(
                world.map, sample_distance_m=args.map_sample_m)
            node.publish_map(graph_dict)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(
            target=executor.spin, name='ros-spin', daemon=True)
        spin_thread.start()
        carla_loop(node, world, args, scenario)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError) as error:
        print('CARLA 场景启动/运行失败：%s' % error, flush=True)
        exit_code = 2
    finally:
        if world is not None:
            world.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
