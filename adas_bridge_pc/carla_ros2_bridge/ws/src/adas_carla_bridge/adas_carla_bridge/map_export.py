#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CARLA OpenDRIVE 地图 → adas_msgs/LaneGraph 的纯 CARLA 侧导出（无 ROS 依赖）。

对应 M6 打通计划第 1 步："CARLA→lane_graph"。地图在一次仿真中不变，
只需在桥启动时导出一次，交给 bridge_node.py 以 transient_local QoS
发布 /adas/map/lane_graph，供 adas_global_planner 订阅规划。

坐标/单位与 carla_world.py 保持一致（右手系，见其模块注释）：
    x_r = x_c   y_r = -y_c   yaw_r = -yaw_c

车道分组：CARLA `Map.generate_waypoints(distance)` 对每条 Driving 车道
按约 distance 米采样，采样点用 (road_id, section_id, lane_id) 三元组
唯一标识所属的物理车道（同一 road_id 因宽度变化被拆成多个 section 时，
视为独立的图节点——与 M6.1 MVP 的"先跑通"目标一致，不强行合并）。

int64 车道 id 编码（位打包，足够覆盖 CARLA 地图规模，避免十进制拼接
在极端地图上溢出/冲突）：
    id = road_id<<24 | (section_id & 0xFF)<<16 | (lane_id+32768 & 0xFFFF)

限速：CARLA waypoint 不直接暴露车道限速（需要另外解析限速地标），
MVP 阶段用固定默认值，后续如需精确限速再接 landmark 查询。
"""

import hashlib
import math

DEFAULT_SAMPLE_DISTANCE_M = 2.0
DEFAULT_SPEED_LIMIT_MPS = 13.9  # ~50 km/h，与 LaneSegment 默认值一致

MANEUVER_STRAIGHT = 0
MANEUVER_LEFT = 1
MANEUVER_RIGHT = 2
MANEUVER_LANE_CHANGE_LEFT = 3
MANEUVER_LANE_CHANGE_RIGHT = 4

# 穿过路口连接段的航向变化超过此阈值才算左/右转，否则视为直行（过路口的直行车道、
# 采样噪声都在此阈值内）。30° 足以区分真实转弯与直行连接段的轻微航向抖动。
TURN_YAW_THRESHOLD_RAD = math.radians(30.0)

# generate_waypoints() 的末采样点通常已靠近车道出口，但不会保证恰好落在边界上。
# 沿 next() 向前扫描少量步数，既能越过同一 lane 内的采样余量，也能防住坏地图环路。
MAX_FORWARD_SCAN_STEPS = 32


def _norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def encode_lane_key(road_id, section_id, lane_id):
    """(road_id, section_id, lane_id) → int64 唯一车道 id（位打包，见模块注释）。"""
    return (int(road_id) << 24) | ((int(section_id) & 0xFF) << 16) | (
        (int(lane_id) + 32768) & 0xFFFF)


def _lane_key(waypoint):
    return (waypoint.road_id, waypoint.section_id, waypoint.lane_id)


def _to_right_handed(transform):
    """CARLA Transform（左手系）→ (x, y, yaw) 右手系，见模块注释。"""
    yaw_c = math.radians(float(transform.rotation.yaw))
    return (float(transform.location.x), -float(transform.location.y), -yaw_c)


def _rh_yaw(waypoint):
    """Waypoint 的右手系航向（rad）：yaw_r = -yaw_c，与 _to_right_handed 一致。"""
    return -math.radians(float(waypoint.transform.rotation.yaw))


def _forward_maneuver(last_wp, dest_wps):
    """判定进入 dest 车道的转向：STRAIGHT / LEFT / RIGHT。

    只有 dest 是路口连接段（is_junction）时才可能判左/右转——普通道路的弯道续接
    仍算直行，避免把高速缓弯误标成转弯让 turn_penalty 乱加。左右由穿过整条连接段的
    航向变化定：右手系里逆时针（+Δyaw）为左转、顺时针（-Δyaw）为右转。取 dest 末点
    航向而非 next() 命中的首点，是为了捕捉整段转弯量，而非路口入口 2m 处的局部航向。
    """
    dest_last = dest_wps[-1]
    if not bool(getattr(dest_last, 'is_junction', False)):
        return MANEUVER_STRAIGHT
    dyaw = _norm_angle(_rh_yaw(dest_last) - _rh_yaw(last_wp))
    if dyaw > TURN_YAW_THRESHOLD_RAD:
        return MANEUVER_LEFT
    if dyaw < -TURN_YAW_THRESHOLD_RAD:
        return MANEUVER_RIGHT
    return MANEUVER_STRAIGHT


def _group_waypoints(waypoints):
    """按物理车道分组并沿行驶方向排序。返回 {lane_key: [waypoint, ...]}。

    OpenDRIVE 右侧车道（负 lane_id）沿 reference line、即 s 递增方向行驶；
    左侧车道（正 lane_id）反向行驶，因此必须按 s 递减排序。CARLA Waypoint.s
    始终是 road reference line 的纵向坐标，不能对两侧车道一律升序。
    """
    groups = {}
    for wp in waypoints:
        groups.setdefault(_lane_key(wp), []).append(wp)
    for key in groups:
        groups[key].sort(key=lambda w: w.s, reverse=key[2] > 0)
    return groups


def _same_direction(a_lane_id, b_lane_id):
    """OpenDRIVE 约定：lane_id 同号 = 同行驶方向（异号跨越道路中心线）。"""
    return (a_lane_id > 0) == (b_lane_id > 0)


def _waypoint_token(waypoint):
    """为前向扫描生成稳定 token；不依赖 CARLA Python 对象是否可哈希。"""
    transform = waypoint.transform
    return (_lane_key(waypoint), round(float(waypoint.s), 6),
            round(float(transform.location.x), 4),
            round(float(transform.location.y), 4))


def _next_waypoints(waypoint, distance_m):
    """调用 CARLA next()，把地图边界异常统一视为无后继。"""
    try:
        return list(waypoint.next(distance_m))
    except RuntimeError:
        return []


def _forward_connections(last_wp, sample_distance_m, groups):
    """last_wp 前方车道 → STRAIGHT/LEFT/RIGHT 连接。

    末采样点的 next(sample_distance) 仍可能落在同一 lane，尤其是短 section 或
    generate_waypoints() 未在边界采样时。此处继续沿行驶方向扫描，直到首次进入图中的
    另一车道；因此既不产生自环，也不会把短连接段的出口拓扑截断。
    """
    connections = []
    source_key = _lane_key(last_wp)
    step_m = max(sample_distance_m, 0.5)
    frontier = [last_wp]
    seen_waypoints = {_waypoint_token(last_wp)}
    seen_destinations = set()

    for _ in range(MAX_FORWARD_SCAN_STEPS):
        next_frontier = []
        for current in frontier:
            for nwp in _next_waypoints(current, step_m):
                token = _waypoint_token(nwp)
                if token in seen_waypoints:
                    continue
                seen_waypoints.add(token)
                key = _lane_key(nwp)

                # 仍在源 lane，或命中了 generate_waypoints() 未包含的中间 lane：
                # 继续扫描，而不是直接丢掉整条分支。
                if key == source_key or key not in groups:
                    next_frontier.append(nwp)
                    continue
                if key in seen_destinations:
                    continue
                seen_destinations.add(key)
                connections.append({
                    'to_key': key,
                    'maneuver': _forward_maneuver(last_wp, groups[key]),
                    'extra_cost_m': 0.0,
                })
        if not next_frontier:
            break
        frontier = next_frontier
    return connections


def _short_lane_centerline(waypoint, sample_distance_m):
    """为只采到一个点的短 lane 补一个前向几何点。

    SoC LaneGraph 要求中心线至少两点。直接删除短路口 connector 会同时删除其入/出边，
    令可达路线断裂。优先使用 CARLA next() 在前方取得真实点；若短 lane 位于地图尽头，
    则沿 waypoint 自带的行驶航向外推 0.5m，保证得到非退化且方向正确的最小线段。
    """
    first = _to_right_handed(waypoint.transform)
    probe_m = min(max(float(sample_distance_m), 0.1), 0.5)
    candidates = []
    for candidate in _next_waypoints(waypoint, probe_m):
        point = _to_right_handed(candidate.transform)
        distance = math.hypot(point[0] - first[0], point[1] - first[1])
        if distance <= 1e-3:
            continue
        heading_delta = abs(_norm_angle(point[2] - first[2]))
        candidates.append((heading_delta, distance, _lane_key(candidate),
                           float(candidate.s), point))
    if candidates:
        # 路口出口可能有多个分支；选航向最连续且最近的一点作为 connector 几何末端。
        return [first, min(candidates, key=lambda item: item[:-1])[-1]]

    length_m = 0.5
    return [first, (first[0] + length_m * math.cos(first[2]),
                    first[1] + length_m * math.sin(first[2]), first[2])]


def _lane_change_connections(last_wp, groups):
    """相邻同向 Driving 车道 → LANE_CHANGE_LEFT/RIGHT 连接（借道变道用）。"""
    connections = []
    lane_change = getattr(last_wp, 'lane_change', None)
    allow_left = str(lane_change) in ('Left', 'Both')
    allow_right = str(lane_change) in ('Right', 'Both')

    if allow_left:
        try:
            left = last_wp.get_left_lane()
        except RuntimeError:
            left = None
        if (left is not None and str(left.lane_type) == 'Driving'
                and _same_direction(left.lane_id, last_wp.lane_id)):
            key = _lane_key(left)
            if key in groups:
                connections.append({
                    'to_key': key,
                    'maneuver': MANEUVER_LANE_CHANGE_LEFT,
                    'extra_cost_m': 0.0,
                })
    if allow_right:
        try:
            right = last_wp.get_right_lane()
        except RuntimeError:
            right = None
        if (right is not None and str(right.lane_type) == 'Driving'
                and _same_direction(right.lane_id, last_wp.lane_id)):
            key = _lane_key(right)
            if key in groups:
                connections.append({
                    'to_key': key,
                    'maneuver': MANEUVER_LANE_CHANGE_RIGHT,
                    'extra_cost_m': 0.0,
                })
    return connections


def build_lane_graph(waypoints, map_name='', sample_distance_m=DEFAULT_SAMPLE_DISTANCE_M,
                     speed_limit_mps=DEFAULT_SPEED_LIMIT_MPS):
    """纯逻辑核心：采样点列表 → LaneGraph 字典。waypoints 只需实现本模块用到的
    CARLA Waypoint 接口子集（road_id/section_id/lane_id/s/transform/lane_width/
    is_junction/lane_type/lane_change/next()/get_left_lane()/get_right_lane()），
    因此可用假对象离线单测，不依赖真实 CARLA 世界。
    """
    groups = _group_waypoints(waypoints)
    lanes = []
    key_to_id = {key: encode_lane_key(*key) for key in groups}

    for key, wps in groups.items():
        lane_id = key_to_id[key]
        if len(wps) == 1:
            centerline = _short_lane_centerline(wps[0], sample_distance_m)
        else:
            centerline = [_to_right_handed(w.transform) for w in wps]
        last_wp = wps[-1]
        outgoing = []
        for conn in (_forward_connections(last_wp, sample_distance_m, groups)
                     + _lane_change_connections(last_wp, groups)):
            outgoing.append({
                'to_lane_id': key_to_id[conn['to_key']],
                'maneuver': conn['maneuver'],
                'extra_cost_m': conn['extra_cost_m'],
            })
        lanes.append({
            'id': lane_id,
            'centerline': centerline,
            'speed_limit_mps': speed_limit_mps,
            'junction': bool(getattr(last_wp, 'is_junction', False)),
            'outgoing': outgoing,
        })

    map_hash = hashlib.sha1(
        ('%s|%d|%.3f' % (map_name, len(lanes), sample_distance_m)).encode('utf-8')
    ).hexdigest()
    return {'map_id': map_name, 'map_hash': map_hash, 'lanes': lanes}


def export_lane_graph(carla_map, sample_distance_m=DEFAULT_SAMPLE_DISTANCE_M,
                      speed_limit_mps=DEFAULT_SPEED_LIMIT_MPS):
    """CARLA 侧入口：真实 carla.Map → LaneGraph 字典。"""
    waypoints = carla_map.generate_waypoints(sample_distance_m)
    return build_lane_graph(waypoints, map_name=carla_map.name,
                            sample_distance_m=sample_distance_m,
                            speed_limit_mps=speed_limit_mps)
