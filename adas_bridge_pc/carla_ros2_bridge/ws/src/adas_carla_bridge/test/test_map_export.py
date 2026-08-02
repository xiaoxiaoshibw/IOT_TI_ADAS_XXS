#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""map_export.build_lane_graph 离线单测：用假 waypoint 验证 LaneGraph 构造，
不依赖 CARLA / ROS。对应 M6 第 1 步的"先写单测验证 lane_graph 构造逻辑"。

覆盖：车道分组、右手系坐标转换、直行连接、变道连接、id 编码唯一性、
断头车道无连接、行驶方向过滤、单点车道过滤、自环连接跳过、路口左/右转分类。
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from adas_carla_bridge.map_export import (build_lane_graph, encode_lane_key)


class FakeRotation:
    def __init__(self, yaw):
        self.yaw = yaw


class FakeLocation:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeTransform:
    def __init__(self, x, y, yaw):
        self.location = FakeLocation(x, y)
        self.rotation = FakeRotation(yaw)


class FakeWaypoint:
    """实现 map_export 用到的 CARLA Waypoint 接口子集。"""

    def __init__(self, road_id, section_id, lane_id, s, x, y, yaw=0.0,
                 lane_width=3.5, is_junction=False, lane_type='Driving',
                 lane_change='NONE'):
        self.road_id = road_id
        self.section_id = section_id
        self.lane_id = lane_id
        self.s = s
        self.transform = FakeTransform(x, y, yaw)
        self.lane_width = lane_width
        self.is_junction = is_junction
        self.lane_type = lane_type
        self.lane_change = lane_change
        self._next = []
        self._left = None
        self._right = None

    def next(self, distance):
        return list(self._next)

    def get_left_lane(self):
        return self._left

    def get_right_lane(self):
        return self._right


def _straight_lane(road_id, lane_id, x0, n=5, step=2.0, yaw=0.0, **kw):
    """沿 +x 生成一条 n 点直车道，返回 waypoint 列表（已按 s 排序）。"""
    wps = []
    for i in range(n):
        wps.append(FakeWaypoint(road_id, 0, lane_id, s=i * step,
                                x=x0 + i * step, y=0.0, yaw=yaw, **kw))
    return wps


def _junction_lane(road_id, lane_id, x0, exit_yaw, n=3, step=2.0):
    """路口连接段：is_junction=True，末点 CARLA 航向 = exit_yaw（度）。

    _forward_maneuver 只读末点航向判转向，中间点航向不影响结果，这里置 0 即可。
    """
    wps = []
    for i in range(n):
        yaw = exit_yaw if i == n - 1 else 0.0
        wps.append(FakeWaypoint(road_id, 0, lane_id, s=i * step,
                                x=x0 + i * step, y=0.0, yaw=yaw,
                                is_junction=True))
    return wps


def test_basic_grouping_and_coords():
    # 一条直车道，road 10 / lane -1，5 个点
    wps = _straight_lane(10, -1, x0=0.0, n=5)
    graph = build_lane_graph(wps, map_name='TestTown', sample_distance_m=2.0)

    assert graph['map_id'] == 'TestTown'
    assert len(graph['lanes']) == 1
    lane = graph['lanes'][0]
    assert lane['id'] == encode_lane_key(10, 0, -1)
    assert len(lane['centerline']) == 5
    # 右手系：y_r = -y_c；这里 y_c=0 → y_r=0
    x, y, yaw = lane['centerline'][0]
    assert abs(x - 0.0) < 1e-9 and abs(y - 0.0) < 1e-9
    # 断头车道（无 next / 无邻道）→ 无连接
    assert lane['outgoing'] == []


def test_y_and_yaw_sign_flip():
    # y_c=4.0, yaw_c=90° → 右手系 y_r=-4.0, yaw_r=-90°
    # 用 2 点车道：单点车道会被 build_lane_graph 过滤，取首点验证坐标/朝向翻转。
    wps = [FakeWaypoint(1, 0, -1, s=0.0, x=3.0, y=4.0, yaw=90.0),
           FakeWaypoint(1, 0, -1, s=2.0, x=3.0, y=6.0, yaw=90.0)]
    graph = build_lane_graph(wps, map_name='M')
    x, y, yaw = graph['lanes'][0]['centerline'][0]
    assert abs(x - 3.0) < 1e-9
    assert abs(y - (-4.0)) < 1e-9
    assert abs(yaw - math.radians(-90.0)) < 1e-9


def test_single_point_lane_dropped():
    # CARLA 对短于采样间距的路口连接线/短 section 只采到 1 点。单点车道无法构成
    # 中心线（SoC add_lane 拒），build_lane_graph 应直接过滤，不能让它进图，
    # 否则订阅端会因这一条坏车道把整张地图判 FAILED。
    good = _straight_lane(10, -1, x0=0.0, n=3)
    stub = [FakeWaypoint(11, 0, -1, s=0.0, x=100.0, y=0.0)]   # 单点车道
    graph = build_lane_graph(good + stub, map_name='M')
    ids = {l['id'] for l in graph['lanes']}
    assert encode_lane_key(10, 0, -1) in ids
    assert encode_lane_key(11, 0, -1) not in ids   # 单点车道被过滤
    assert len(graph['lanes']) == 1


def test_forward_connection_to_dropped_lane_skipped():
    # 指向被过滤的单点车道的前向连接也应一并消失（否则订阅端会拒该连接）。
    lane_a = _straight_lane(10, -1, x0=0.0, n=3)
    stub = [FakeWaypoint(11, 0, -1, s=0.0, x=6.0, y=0.0)]   # 单点车道
    lane_a[-1]._next = [stub[0]]   # a 末端指向被过滤的单点车道
    graph = build_lane_graph(lane_a + stub, map_name='M')
    a_id = encode_lane_key(10, 0, -1)
    lanes = {l['id']: l for l in graph['lanes']}
    assert lanes[a_id]['outgoing'] == []   # 无指向已过滤车道的连接


def test_self_loop_forward_connection_skipped():
    # 长直车道末点 next(2m) 可能仍落在同一 (road,section,lane) → from==to 自环，
    # SoC add_connection 会拒。_forward_connections 应直接跳过自环。
    lane = _straight_lane(10, -1, x0=0.0, n=3)
    lane[-1]._next = [lane[-1]]   # 末点 next 落回自身车道
    graph = build_lane_graph(lane, map_name='M')
    a_id = encode_lane_key(10, 0, -1)
    lanes = {l['id']: l for l in graph['lanes']}
    assert all(c['to_lane_id'] != a_id for c in lanes[a_id]['outgoing'])
    assert lanes[a_id]['outgoing'] == []


def _forward_maneuver_into_junction(exit_yaw_c):
    """进入一条末点 CARLA 航向 = exit_yaw_c 的路口连接段，返回该前向连接的 maneuver。"""
    approach = _straight_lane(10, -1, x0=0.0, n=3)   # 直行进入，yaw_c=0
    junction = _junction_lane(20, -1, x0=6.0, exit_yaw=exit_yaw_c, n=3)
    approach[-1]._next = [junction[0]]
    graph = build_lane_graph(approach + junction, map_name='M')
    lanes = {l['id']: l for l in graph['lanes']}
    conns = {c['to_lane_id']: c['maneuver']
             for c in lanes[encode_lane_key(10, 0, -1)]['outgoing']}
    return conns[encode_lane_key(20, 0, -1)]


def test_forward_left_turn_classified():
    # CARLA 末点航向 -90° → 右手系 +90°（逆时针）→ 左转
    assert _forward_maneuver_into_junction(-90.0) == 1   # MANEUVER_LEFT


def test_forward_right_turn_classified():
    # CARLA 末点航向 +90° → 右手系 -90°（顺时针）→ 右转
    assert _forward_maneuver_into_junction(90.0) == 2    # MANEUVER_RIGHT


def test_forward_through_junction_is_straight():
    # 过路口但航向几乎不变（10° < 30° 阈值）→ 直行
    assert _forward_maneuver_into_junction(-10.0) == 0   # MANEUVER_STRAIGHT


def test_forward_nonjunction_curve_is_straight():
    # 目标不是路口连接段：即便末点航向转了 90°，也按直行处理（不误标转弯）
    approach = _straight_lane(10, -1, x0=0.0, n=3)
    curve = _straight_lane(20, -1, x0=6.0, n=3, yaw=-90.0)   # 非路口弯道
    approach[-1]._next = [curve[0]]
    graph = build_lane_graph(approach + curve, map_name='M')
    lanes = {l['id']: l for l in graph['lanes']}
    conns = {c['to_lane_id']: c['maneuver']
             for c in lanes[encode_lane_key(10, 0, -1)]['outgoing']}
    assert conns[encode_lane_key(20, 0, -1)] == 0   # MANEUVER_STRAIGHT


def test_forward_straight_connection():
    lane_a = _straight_lane(10, -1, x0=0.0, n=3)
    lane_b = _straight_lane(11, -1, x0=6.0, n=3)
    lane_a[-1]._next = [lane_b[0]]   # a 末端 → b 起点

    graph = build_lane_graph(lane_a + lane_b, map_name='M', sample_distance_m=2.0)
    lanes = {l['id']: l for l in graph['lanes']}
    a_id = encode_lane_key(10, 0, -1)
    b_id = encode_lane_key(11, 0, -1)
    conns = lanes[a_id]['outgoing']
    assert len(conns) == 1
    assert conns[0]['to_lane_id'] == b_id
    assert conns[0]['maneuver'] == 0   # STRAIGHT
    # b 是断头 → 无连接
    assert lanes[b_id]['outgoing'] == []


def test_lane_change_connections():
    # 同 road、相邻同向车道 -1 / -2，允许双向变道
    lane1 = _straight_lane(10, -1, x0=0.0, n=3, lane_change='Both')
    lane2 = _straight_lane(10, -2, x0=0.0, n=3, lane_change='Both')
    # 让每条车道的末端点互指为左右邻道
    lane1[-1]._right = lane2[-1]
    lane2[-1]._left = lane1[-1]

    graph = build_lane_graph(lane1 + lane2, map_name='M')
    lanes = {l['id']: l for l in graph['lanes']}
    id1 = encode_lane_key(10, 0, -1)
    id2 = encode_lane_key(10, 0, -2)

    m1 = {c['to_lane_id']: c['maneuver'] for c in lanes[id1]['outgoing']}
    m2 = {c['to_lane_id']: c['maneuver'] for c in lanes[id2]['outgoing']}
    assert m1.get(id2) == 4   # -1 → -2 是 LANE_CHANGE_RIGHT
    assert m2.get(id1) == 3   # -2 → -1 是 LANE_CHANGE_LEFT


def test_opposite_direction_not_connected():
    # 相邻但异向（lane_id 异号）→ 不应产生变道连接
    lane_neg = _straight_lane(10, -1, x0=0.0, n=3, lane_change='Both')
    lane_pos = _straight_lane(10, 1, x0=0.0, n=3, lane_change='Both')
    lane_neg[-1]._left = lane_pos[-1]

    graph = build_lane_graph(lane_neg + lane_pos, map_name='M')
    lanes = {l['id']: l for l in graph['lanes']}
    id_neg = encode_lane_key(10, 0, -1)
    assert lanes[id_neg]['outgoing'] == []


def test_non_driving_lane_change_skipped():
    lane = _straight_lane(10, -1, x0=0.0, n=3, lane_change='Both')
    shoulder = _straight_lane(10, -2, x0=0.0, n=3, lane_type='Shoulder')
    lane[-1]._right = shoulder[-1]

    graph = build_lane_graph(lane + shoulder, map_name='M')
    lanes = {l['id']: l for l in graph['lanes']}
    id1 = encode_lane_key(10, 0, -1)
    # 右邻是 Shoulder（非 Driving）→ 无变道连接
    assert lanes[id1]['outgoing'] == []


def test_id_encoding_unique():
    keys = [(10, 0, -1), (10, 0, -2), (10, 1, -1), (11, 0, -1), (10, 0, 1)]
    ids = [encode_lane_key(*k) for k in keys]
    assert len(set(ids)) == len(keys)   # 无冲突


def test_map_hash_stable_and_sensitive():
    wps = _straight_lane(10, -1, x0=0.0, n=3)
    h1 = build_lane_graph(wps, map_name='A', sample_distance_m=2.0)['map_hash']
    h2 = build_lane_graph(wps, map_name='A', sample_distance_m=2.0)['map_hash']
    h3 = build_lane_graph(wps, map_name='B', sample_distance_m=2.0)['map_hash']
    assert h1 == h2       # 稳定
    assert h1 != h3       # 对地图名敏感


if __name__ == '__main__':
    import traceback
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    failed = 0
    for fn in tests:
        try:
            fn()
            print('PASS %s' % fn.__name__)
        except Exception:
            failed += 1
            print('FAIL %s' % fn.__name__)
            traceback.print_exc()
    print('\n%d/%d passed' % (len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)
