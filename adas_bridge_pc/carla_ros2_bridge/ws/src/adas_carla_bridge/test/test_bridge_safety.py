import math
import json
import time
from types import SimpleNamespace
import uuid

import pytest
import rclpy
from adas_msgs.msg import ActuationCommand
from std_msgs.msg import String

from adas_carla_bridge.bridge_node import (
    CarlaBridgeNode,
    _spin_executor,
    build_arg_parser,
    main,
    validate_actuation_values,
)


def test_executor_external_shutdown_is_a_clean_exit():
    from rclpy.executors import ExternalShutdownException

    class ShuttingDownExecutor:
        def spin(self):
            raise ExternalShutdownException()

    _spin_executor(ShuttingDownExecutor())


def test_scenario_cli_accepts_file_seed_and_expected_count():
    args = build_arg_parser().parse_args([
        '--scenario-file', 'scenarios/dense_overtake_v1.json',
        '--seed', '7', '--expected-actor-count', '20'])

    assert args.scenario_file.endswith('dense_overtake_v1.json')
    assert args.seed == 7
    assert args.expected_actor_count == 20


def test_negative_expected_actor_count_fails_before_carla_connect():
    assert main(['--expected-actor-count', '-1']) == 2


def test_valid_actuation_is_accepted():
    assert validate_actuation_values(0.5, 0.0, -0.25) == (True, 'ok')
    assert validate_actuation_values(0.0, 1.0, 1.0) == (True, 'ok')


def test_non_finite_actuation_is_rejected():
    for value in (math.nan, math.inf, -math.inf):
        assert not validate_actuation_values(value, 0.0, 0.0)[0]
        assert not validate_actuation_values(0.0, value, 0.0)[0]
        assert not validate_actuation_values(0.0, 0.0, value)[0]


def test_out_of_range_actuation_is_rejected():
    assert not validate_actuation_values(-0.01, 0.0, 0.0)[0]
    assert not validate_actuation_values(1.01, 0.0, 0.0)[0]
    assert not validate_actuation_values(0.0, -0.01, 0.0)[0]
    assert not validate_actuation_values(0.0, 1.01, 0.0)[0]
    assert not validate_actuation_values(0.0, 0.0, -1.01)[0]
    assert not validate_actuation_values(0.0, 0.0, 1.01)[0]


def test_throttle_brake_conflict_is_rejected():
    valid, reason = validate_actuation_values(0.2, 0.3, 0.0)
    assert not valid
    assert reason == 'throttle_brake_conflict'


def test_invalid_frame_latches_failsafe_until_three_valid_frames():
    if not rclpy.ok():
        rclpy.init()
    node = CarlaBridgeNode(SimpleNamespace(
        stale_timeout_s=0.5, control_source='ros2', scenario='test',
        run_id='11111111-2222-4333-8444-555555555555'))
    try:
        invalid = ActuationCommand()
        invalid.throttle = math.nan
        node._actuation_cb(invalid)
        assert node.get_actuation()['invalid_latched']
        assert node.get_actuation()['brake'] == 1.0

        valid = ActuationCommand()
        valid.throttle = 0.2
        valid.brake = 0.0
        valid.steer = 0.1
        for _ in range(2):
            node._actuation_cb(valid)
            assert node.get_actuation()['invalid_latched']
        node._actuation_cb(valid)
        recovered = node.get_actuation()
        assert not recovered['invalid_latched']
        assert recovered['throttle'] == 0.2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fault_ack_requires_uuid4_and_reports_can_contract():
    class Receiver:
        def send_fault_injection(self, command, parameter, sequence):
            assert (command, parameter, sequence) == (3, 9, 1)
            return {'system_state': 4, 'fault_level': 1, 'sequence': sequence}

        def close(self):
            pass

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(json.loads(message.data))

    rclpy.init()
    node = CarlaBridgeNode(SimpleNamespace(
        stale_timeout_s=0.5, control_source='ros2', scenario='test',
        run_id='11111111-2222-4333-8444-555555555555'))
    publisher = Publisher()
    node.pub_fault_ack = publisher
    node._can_receiver = Receiver()
    try:
        invalid = String()
        invalid.data = json.dumps({'request_id': 'not-a-uuid', 'cmd': 3, 'param': 9})
        node._fault_inject_cb(invalid)
        assert not publisher.messages[-1]['accepted']

        request_id = str(uuid.uuid4())
        valid = String()
        valid.data = json.dumps({'request_id': request_id, 'cmd': 3, 'param': 9})
        node._fault_inject_cb(valid)
        ack = publisher.messages[-1]
        assert ack['accepted']
        assert ack['request_id'] == request_id
        assert ack['can_id'] == '0x302'
        assert ack['dlc'] == 8
        assert ack['crc_valid']
        assert ack['param'] == 9
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fault_ack_requires_uuid4_and_reports_can_contract():
    class Receiver:
        def send_fault_injection(self, command, parameter, sequence):
            assert (command, parameter, sequence) == (3, 9, 1)
            return {'system_state': 4, 'fault_level': 1, 'sequence': sequence}

        def close(self):
            pass

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(json.loads(message.data))

    if not rclpy.ok():
        rclpy.init()
    node = CarlaBridgeNode(SimpleNamespace(
        stale_timeout_s=0.5, control_source='ros2', scenario='test',
        run_id='11111111-2222-4333-8444-555555555555'))
    """P0.C: 消费端 _accept_run_id 必须拒绝空 run_id 与错配会话。"""
    if not rclpy.ok():
        rclpy.init()
    args = SimpleNamespace(stale_timeout_s=0.5, control_source='ros2',
                           scenario='test',
                           run_id='11111111-2222-4333-8444-555555555555')
    node = CarlaBridgeNode(args)
    try:
        # 当前会话内消息必须放行
        assert node._accept_run_id('11111111-2222-4333-8444-555555555555')
        # 空 run_id 一律拒绝（无通配语义）
        assert not node._accept_run_id('')
        # 旧会话残留必须拒绝
        assert not node._accept_run_id('aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee')
        assert not node._accept_run_id('stale:handshake:1')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_run_id_auto_generates_only_with_explicit_flag():
    """P0.C: --run-id 为空 + --auto-run-id 才会自动生成 UUID v4;
    否则 fail-closed,防止 GUI/Orchestrator 误传空值导致两端 ID 互不识别。"""
    # 必须先 init,否则 Node 创建会直接报错;测试用例要的是"构造函数
    # 自己 SystemExit",但 Node 创建需要 rclpy 已初始化。
    if not rclpy.ok():
        rclpy.init()
    # 缺省 --auto-run-id: 构造函数必须直接 SystemExit,run_id 不会被设置。
    with pytest.raises(SystemExit):
        CarlaBridgeNode(SimpleNamespace(
            stale_timeout_s=0.5, control_source='ros2',
            scenario='test', run_id=''))
    # 显式 --auto-run-id: 自动生成 UUID v4,且可被消费端过滤使用。
    node = CarlaBridgeNode(SimpleNamespace(
        stale_timeout_s=0.5, control_source='ros2',
        scenario='test', run_id='', auto_run_id=True))
    try:
        parsed = uuid.UUID(node._current_run_id)
        assert parsed.version == 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_run_id_rejects_non_uuid_v4():
    """P0.C: --run-id 必须是规范 UUID v4,否则必须 fail-closed。"""
    if not rclpy.ok():
        rclpy.init()
    with pytest.raises(SystemExit):
        CarlaBridgeNode(SimpleNamespace(
            stale_timeout_s=0.5, control_source='ros2',
            scenario='test', run_id='not-a-uuid'))


def test_legacy_path_callback_removed():
    """P0.3: bridge 不再订阅 /adas/planning/global_route,避免旧会话 Path 绕回。"""
    if not rclpy.ok():
        rclpy.init()
    node = CarlaBridgeNode(SimpleNamespace(
        stale_timeout_s=0.5, control_source='ros2',
        scenario='test', run_id='11111111-2222-4333-8444-555555555555'))
    try:
        assert not hasattr(node, '_legacy_path_route_cb')
        # _route_source_locked 是被删掉的旧双发标志位
        assert '_route_source_locked' not in node._visual
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_freshness_watchdog_clears_stale_route_and_aeb():
    """P0.D: 超过 freshness 阈值的 route/goal/AEB 必须被看门狗清空。"""
    if not rclpy.ok():
        rclpy.init()
    args = SimpleNamespace(stale_timeout_s=0.5, control_source='ros2',
                           scenario='test',
                           run_id='11111111-2222-4333-8444-555555555555')
    node = CarlaBridgeNode(args)
    try:
        node._freshness_timeout_s = 0.1  # 压缩等待时间
        # P0.D: seen_t=0.0 等价于"未观察过"；测试想表达"很久以前"，故用极小正数。
        ancient = 1e-6
        with node._visual_lock:
            node._visual['route'] = [(1.0, 2.0), (3.0, 4.0)]
            node._visual['route_seen_t'] = ancient
            node._visual['pending_goal'] = (5.0, 6.0)
            node._visual['goal_seen_t'] = ancient
            node._visual['aeb_state'] = 2
            node._visual['aeb_seen_t'] = ancient
            node._visual['behavior_state'] = 3
            node._visual['behavior_seen_t'] = ancient
        node._freshness_watchdog()
        with node._visual_lock:
            assert node._visual['route'] == []
            assert node._visual['pending_goal'] is None
            assert node._visual['aeb_state'] == 0
            assert node._visual['behavior_state'] == -1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_freshness_watchdog_keeps_fresh_values():
    """P0.D: 在 freshness 阈值内的字段必须保持原值。"""
    if not rclpy.ok():
        rclpy.init()
    args = SimpleNamespace(stale_timeout_s=0.5, control_source='ros2',
                           scenario='test',
                           run_id='11111111-2222-4333-8444-555555555555')
    node = CarlaBridgeNode(args)
    try:
        node._freshness_timeout_s = 5.0  # 阈值很大
        with node._visual_lock:
            node._visual['route'] = [(1.0, 2.0)]
            node._visual['route_seen_t'] = time.monotonic()
            node._visual['aeb_state'] = 2
            node._visual['aeb_seen_t'] = time.monotonic()
        node._freshness_watchdog()
        with node._visual_lock:
            assert node._visual['route'] == [(1.0, 2.0)]
            assert node._visual['aeb_state'] == 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_close_is_idempotent_and_clears_visual_cache():
    """P0.D: 桥节点 close 多次调用必须安全，且 _visual 缓存必须清空。"""
    if not rclpy.ok():
        rclpy.init()
    args = SimpleNamespace(stale_timeout_s=0.5, control_source='ros2',
                           scenario='test',
                           run_id='11111111-2222-4333-8444-555555555555')
    node = CarlaBridgeNode(args)
    try:
        with node._visual_lock:
            node._visual['route'] = [(1.0, 2.0)]
            node._visual['pending_goal'] = (3.0, 4.0)
            node._visual['behavior_state'] = 1
        # 第一次 close：必须不报错
        node.close()
        # 第二次 close：必须不报错
        node.close()
        with node._visual_lock:
            assert node._visual['route'] == []
            assert node._visual['pending_goal'] is None
            assert node._visual['behavior_state'] == -1
            assert all(
                node._visual[k] == 0.0
                for k in ('route_seen_t', 'status_seen_t', 'behavior_seen_t',
                          'aeb_seen_t', 'safety_seen_t', 'goal_seen_t'))
    finally:
        node.destroy_node()
        rclpy.shutdown()
