#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CARLA 世界端：连接/生成/真值感知/执行写回（无 ROS 依赖，纯 CARLA Python API）。

坐标约定（与 ADAS0.0.2 SoC 栈对齐，见 adas_msgs/LaneState.msg 注释）：
  CARLA 用 UE4 左手系（x 前、y 右、z 上、yaw 顺时针为正）；
  SoC 栈用标准右手系（REP-103：左正、逆时针为正、左转曲率为正）。
  本模块在边界处一次性转换，对外输出的 frame 全部是右手系：
      x_r = x_c        y_r = -y_c        yaw_r = -yaw_c
      横向偏移左正     航向误差左正      曲率左转正
      yaw_rate_r = -angular_velocity_z_c
  执行方向：ActuationCommand.steer 左正 → CARLA steer 右正 → 写入时取反。

参考线跟踪（沿用 HIL 桥的教训）：map.get_waypoint 会吸附"最近车道"，
变道/借道跨线瞬间 lane_offset 突跳；因此锚定自车初始车道中心线，
按纵向投影推进参考 waypoint，只有偏离 >20m 才重新吸附。
SoC 栈的 trajectory_planner 恰好也以"同一根中心线 + 横向偏移"规划变道，
两端假设一致。
"""

import math
import uuid
from dataclasses import dataclass

CARLA_TIMEOUT_S = 60.0
SPAWN_WAYPOINT_MAX_DIST_M = 4.0
STATION_WAYPOINT_STEP_M = 5.0
# Phase 4 hardening：自车 / 前车 / 行人 互相之间最小生成距离 (m)。
# 防止多车场景下 spawn 撞墙 / 互相重叠。spawn 时若任一已有 actor
# 与目标位置距离 < 该值,raise RuntimeError 让上层捕获并回退。
SAFE_SPAWN_DISTANCE_M = 5.0
# spawn 后自检：actor 离地高度 < 该值视为穿地 / 撞墙。
SPAWN_AUDIT_MIN_HEIGHT_M = 0.0
# spawn 后自检：actor 与 hero 距离 < 该值视为重叠。
SPAWN_AUDIT_MIN_HERO_DIST_M = 1.0
EGO_BLUEPRINT = 'vehicle.tesla.model3'
LEAD_BLUEPRINT = 'vehicle.audi.tt'
WALKER_BLUEPRINT = 'walker.pedestrian.*'

# adas_msgs/TrackedObject 分类常量（与 msg 定义一致，避免此处依赖 ROS）
CLASS_CAR = 1
CLASS_PEDESTRIAN = 3
CLASS_BICYCLE = 4
CLASS_UNKNOWN = 0

CLASSIFICATION_IDS = {
    'unknown': CLASS_UNKNOWN,
    'vehicle': CLASS_CAR,
    'pedestrian': CLASS_PEDESTRIAN,
    'cyclist': CLASS_BICYCLE,
}

# 曲率估计的前视弧长 [m]。
# 决策：导航场景下若车辆仍在直道、但 next(CURVATURE_LOOKAHEAD_M) 已跨入弯道，
# _ref_curvature 会提前报告非零曲率；trajectory_planner 把它当成整段 120 m
# 轨迹的常曲率外推，纯跟踪（lookahead≈12 m @ 15 m/s）便在到达几何转弯点之前
# 提前打方向——用户报告的"还没到转弯点就提前转弯"即由此放大。
# 2 m 把前视收紧到"接近自车"：仅当车辆即将到达或已在弯内时曲率才非零，
# 平滑性由 trajectory_planner 与 pure_pursuit 的 lookahead 自然保证。
# 该常量仅影响 CARLA 模式；SIL (adas_sim_vehicle) 与 HIL (真实感知) 不依赖此值。
CURVATURE_LOOKAHEAD_M = 2.0
REF_BRANCH_MAX_HEADING_ERROR_RAD = math.radians(45.0)
LANE_INVALID_HEADING_ERROR_RAD = math.radians(45.0)
LANE_INVALID_LATERAL_OFFSET_M = 1.5


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _speed(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _xy_distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _waypoint_yaw(wp):
    return math.radians(float(wp.transform.rotation.yaw))


def _select_forward_waypoint(candidates, ego_loc, ego_yaw, ref_yaw):
    """Choose a junction successor without jumping onto a crossing lane."""
    aligned = []
    for candidate in candidates:
        yaw = _waypoint_yaw(candidate)
        ego_error = abs(_norm_angle(yaw - ego_yaw))
        ref_error = abs(_norm_angle(yaw - ref_yaw))
        if (ego_error <= REF_BRANCH_MAX_HEADING_ERROR_RAD and
                ref_error <= REF_BRANCH_MAX_HEADING_ERROR_RAD):
            distance_sq = (
                (candidate.transform.location.x - ego_loc.x) ** 2 +
                (candidate.transform.location.y - ego_loc.y) ** 2)
            aligned.append((distance_sq + 4.0 * ego_error * ego_error,
                            candidate))
    return min(aligned, key=lambda item: item[0])[1] if aligned else None


def _lane_reference_valid(lateral_offset, heading_error):
    return not (
        abs(heading_error) > LANE_INVALID_HEADING_ERROR_RAD and
        abs(lateral_offset) > LANE_INVALID_LATERAL_OFFSET_M)


@dataclass
class ScriptedActor:
    """One actor owned by this CarlaWorld run and its deterministic state."""

    actor_id: int
    classification: int
    actor: object
    config: dict
    reference_waypoint: object
    velocity_integral: float = 0.0
    triggered: bool = False

    def target_speed(self, sim_t):
        speed = 0.0
        for time_s, target_speed in self.config['speed_profile']:
            if sim_t >= float(time_s):
                speed = float(target_speed)
        return speed

    def in_hard_brake(self, sim_t):
        window = self.config.get('hard_brake_window_s')
        return bool(window) and float(window[0]) <= sim_t <= float(window[1])


class CarlaWorld:
    """持有 CARLA 客户端与场景演员，输出右手系真值帧、接收归一化执行量。"""

    def __init__(self, carla, host, port, scenario, town='Town04',
                 fixed_dt=0.02, no_rendering=False, spawn_index=None):
        self.carla = carla
        self.scenario = scenario
        self.fixed_dt = float(fixed_dt)
        self.no_rendering = bool(no_rendering)

        self.client = carla.Client(host, port)
        self.client.set_timeout(CARLA_TIMEOUT_S)
        cur = self.client.get_world()
        if town and town not in cur.get_map().name:
            print('loading map %s ...' % town, flush=True)
            self.world = self.client.load_world(town)
        else:
            self.world = cur
        self.map = self.world.get_map()
        self.original_settings = self.world.get_settings()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.fixed_dt
        settings.no_rendering_mode = bool(no_rendering)
        self.world.apply_settings(settings)

        if spawn_index is None:
            spawn_index = int(scenario.get(
                'ego', {}).get('spawn_index', scenario.get('spawn_index', 30)))

        self.run_marker = uuid.uuid4().hex
        self.spawned = []
        self.scripted_actors = []
        self.ego = None
        self.lead = None
        self.walker = None
        self.sim_t0 = None
        self._last_steer_cmd = 0.0    # 左正（右手系），转角回读兜底用
        self._last_visualization_t = float('-inf')
        try:
            self.ego = self._spawn(
                EGO_BLUEPRINT, self._owned_role('ego'), spawn_index)
            self.world.tick()

            ego_wp = self._driving_waypoint(
                self.ego.get_transform().location, project_to_road=True)
            if ego_wp is None:
                raise RuntimeError('无法获取自车生成车道')
            self._spawn_scripted_actors(ego_wp)

            phys = self.ego.get_physics_control()
            self.max_steer_rad = math.radians(
                max(w.max_steer_angle for w in phys.wheels) or 70.0)

            # 参考中心线：锚定自车初始车道
            self._ref_wp = self._driving_waypoint(
                self.ego.get_transform().location, project_to_road=False)
            if self._ref_wp is None:
                self._ref_wp = self._driving_waypoint(
                    self.ego.get_transform().location, project_to_road=True)
            if self._ref_wp is None:
                raise RuntimeError('自车不在可驾驶车道附近，无法初始化参考线')

            # Phase 4 hardening：warmup tick 从 5 次降到 2 次,加速启动。
            # 5 次 tick 在 sync_mode 下约 250ms,实测 2 次足够让物理引擎
            # 稳定 actor 位置（不会撞墙）。
            for _ in range(2):
                self.world.tick()
            # spawn audit：检查所有 spawned actor 离地高度、与 hero 距离。
            self._audit_actors()
        except Exception as error:
            cleanup_error = None
            try:
                self._destroy_spawned(flush=True)
            except RuntimeError as caught:
                cleanup_error = caught
            try:
                self.world.apply_settings(self.original_settings)
            except RuntimeError:
                pass
            if cleanup_error is not None:
                raise RuntimeError('%s; cleanup failed: %s' %
                                   (error, cleanup_error)) from error
            raise

    # ── 生成/清理 ──

    def _driving_waypoint(self, location, project_to_road):
        try:
            return self.map.get_waypoint(
                location, project_to_road=project_to_road,
                lane_type=self.carla.LaneType.Driving)
        except RuntimeError:
            return None

    def _owned_role(self, kind, actor_id=None):
        suffix = kind if actor_id is None else '%s:%d' % (kind, actor_id)
        return 'adas:%s:%s' % (self.run_marker, suffix)

    def _destroy_actor_batch(self, actors, flush=False):
        actors = list(actors)
        actor_ids = {
            int(actor.id) for actor in actors if hasattr(actor, 'id')}
        for actor in reversed(actors):
            try:
                if bool(getattr(actor, 'is_alive', True)):
                    actor.destroy()
            except RuntimeError:
                pass
        if not flush or not actor_ids or not hasattr(self, 'world'):
            return

        self.world.tick()
        remaining = {
            int(actor.id): actor for actor in self.world.get_actors()
            if int(actor.id) in actor_ids
        }
        if remaining:
            for actor in remaining.values():
                try:
                    actor.destroy()
                except RuntimeError:
                    pass
            self.world.tick()
            remaining = {
                int(actor.id) for actor in self.world.get_actors()
                if int(actor.id) in actor_ids
            }
        if remaining:
            raise RuntimeError('owned actor cleanup incomplete: %s' %
                               sorted(remaining))

    def _destroy_spawned(self, flush=False):
        self._destroy_actor_batch(self.spawned, flush=flush)
        self.spawned.clear()

    def _aligned_spawn_transform(self, transform):
        wp = self._driving_waypoint(transform.location, project_to_road=False)
        if wp is None:
            wp = self._driving_waypoint(transform.location, project_to_road=True)
        if wp is None:
            return None
        # Phase 4 hardening：拒绝非 Driving 车道（停车道、行人道等）。如果
        # 目标点投影到了步行道或停车道,直接 None,让 _spawn 顺延到下一个点。
        # 这里只读 wp.lane_type 属性,不依赖 CARLA 的 waypoint.is_driving 这种
        # 较新 API（0.9.16 也支持但有些地图返回不一致）。
        try:
            if int(wp.lane_type) != int(self.carla.LaneType.Driving):
                return None
        except (AttributeError, RuntimeError):
            return None
        if _xy_distance(transform.location, wp.transform.location) > SPAWN_WAYPOINT_MAX_DIST_M:
            return None
        loc = self.carla.Location(
            x=float(wp.transform.location.x),
            y=float(wp.transform.location.y),
            z=float(transform.location.z))
        rot = self.carla.Rotation(
            pitch=float(transform.rotation.pitch),
            yaw=float(wp.transform.rotation.yaw),
            roll=float(transform.rotation.roll))
        return self.carla.Transform(loc, rot)

    def _too_close_to_existing(self, transform):
        """检查目标 transform 是否与已有 actor 距离过近。

        Phase 4 hardening:多车场景下防止 spawn 互相重叠 / 撞墙。
        返回 True 表示"太近了,需要顺延到下一个 spawn point"。
        """
        for existing in self.spawned:
            try:
                loc = existing.get_location()
                if _xy_distance(loc, transform.location) < SAFE_SPAWN_DISTANCE_M:
                    return True
            except RuntimeError:
                continue
        return False

    def _spawn(self, bp_name, role, spawn_index):
        bp = self.world.get_blueprint_library().find(bp_name)
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', role)
        points = self.map.get_spawn_points()
        if not points:
            raise RuntimeError('地图没有生成点')
        n = len(points)
        for off in range(n):
            idx = (spawn_index + off) % n
            transform = self._aligned_spawn_transform(points[idx])
            if transform is None:
                continue
            # Phase 4 hardening：与已有 actor 距离过近则跳过该点,顺延到下一个。
            # 这样多车场景下不会发生"hero / lead / walker 三车撞在一起"。
            if self._too_close_to_existing(transform):
                continue
            actor = self.world.try_spawn_actor(bp, transform)
            if actor is not None:
                if off != 0:
                    print('spawn_index %d 不可用，已顺延到 %d'
                          % (spawn_index, idx), flush=True)
                self.spawned.append(actor)
                return actor
        raise RuntimeError('无法生成 %s' % bp_name)

    def _blueprint(self, bp_name):
        lib = self.world.get_blueprint_library()
        if '*' in bp_name:
            matches = list(lib.filter(bp_name))
            if not matches:
                raise RuntimeError('找不到蓝图 %s' % bp_name)
            return matches[0]
        return lib.find(bp_name)

    def _spawn_at(self, bp_name, role, transform, actor_type='vehicle'):
        bp = self._blueprint(bp_name)
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', role)
        loc = self.carla.Location(
            x=float(transform.location.x), y=float(transform.location.y),
            z=float(transform.location.z) +
            (0.2 if actor_type == 'pedestrian' else 0.5))
        rot = self.carla.Rotation(
            pitch=float(transform.rotation.pitch),
            yaw=float(transform.rotation.yaw),
            roll=float(transform.rotation.roll))
        spawn_transform = self.carla.Transform(loc, rot)
        # Phase 4 hardening：目标位置距离已有 actor 过近则 raise,
        # 让上层选择别的位置而不是 silently 重叠。
        if self._too_close_to_existing(spawn_transform):
            raise RuntimeError(
                '目标位置与已有 actor 距离 < %.1fm (SAFE_SPAWN_DISTANCE_M),'
                '拒绝 spawn 以避免撞墙/重叠' % SAFE_SPAWN_DISTANCE_M)
        actor = self.world.try_spawn_actor(bp, spawn_transform)
        if actor is None:
            raise RuntimeError('无法生成目标 %s role=%s' % (bp_name, role))
        self.spawned.append(actor)
        return actor

    def _audit_actors(self):
        """spawn 后自检：所有 spawned actor 必须落在地面、与 hero 不重叠。

        检查项：
          1. location.z >= SPAWN_AUDIT_MIN_HEIGHT_M（CARLA 坐标 z=0 是地面,
             负值意味穿地 = 撞墙/被卡在墙里）
          2. 与 hero 距离 >= SPAWN_AUDIT_MIN_HERO_DIST_M（防止 spawn 重叠）

        Phase 4 hardening：发现问题仅 print 警告,不 raise —— 因为某些场景
        故意要让车静止 / 在桥下,但给操作员明确的可见信号。
        """
        if not self.spawned:
            return
        hero = self.ego
        issues = []
        for actor in self.spawned:
            try:
                loc = actor.get_location()
            except RuntimeError:
                continue
            if loc.z < SPAWN_AUDIT_MIN_HEIGHT_M:
                issues.append(
                    '%s(id=%s) z=%.2f < %.1fm，可能穿地/撞墙'
                    % (actor.type_id, actor.id, loc.z, SPAWN_AUDIT_MIN_HEIGHT_M))
            if hero is not None and actor.id != hero.id:
                try:
                    hero_loc = hero.get_location()
                    d = _xy_distance(loc, hero_loc)
                    if d < SPAWN_AUDIT_MIN_HERO_DIST_M:
                        issues.append(
                            '%s(id=%s) 与 hero 距离 %.2fm < %.1fm,可能重叠'
                            % (actor.type_id, actor.id, d,
                               SPAWN_AUDIT_MIN_HERO_DIST_M))
                except RuntimeError:
                    continue
        if issues:
            print('[AUDIT] spawn 后自检发现问题:', flush=True)
            for line in issues:
                print('  -', line, flush=True)

    def _waypoint_at_station(self, origin, station_m):
        if abs(station_m) < 1e-6:
            return origin
        current = origin
        remaining = abs(float(station_m))
        while remaining > 1e-6:
            step = min(remaining, STATION_WAYPOINT_STEP_M)
            candidates = (current.next(step) if station_m > 0.0
                          else current.previous(step))
            if not candidates:
                raise RuntimeError(
                    '无法在 station %.1fm 找到生成点' % station_m)
            current_yaw = _waypoint_yaw(current)
            selected = _select_forward_waypoint(
                candidates, current.transform.location,
                current_yaw, current_yaw)
            if selected is None:
                raise RuntimeError(
                    'station %.1fm 的局部生成点方向不一致' % station_m)
            current = selected
            remaining -= step
        return current

    def _spawn_scripted_actors(self, ego_waypoint):
        configs = list(self.scenario.get('actors', []))
        actor_ids = [int(config['id']) for config in configs]
        if actor_ids != sorted(actor_ids) or len(actor_ids) != len(set(actor_ids)):
            raise RuntimeError('scripted actor ID 必须唯一且升序')
        initial_spawned = len(self.spawned)
        initial_scripted = len(self.scripted_actors)
        try:
            for config in configs:
                actor_id = int(config['id'])
                classification_name = str(config['classification'])
                if classification_name not in CLASSIFICATION_IDS:
                    raise RuntimeError(
                        'actor %d classification 不支持' % actor_id)
                station = float(config['initial_station_m'])
                waypoint = self._waypoint_at_station(ego_waypoint, station)
                lateral_left = float(config['initial_lateral_m'])
                transform = self._offset_transform_carla(
                    waypoint.transform, -lateral_left)
                actor_type = (
                    'pedestrian' if classification_name == 'pedestrian'
                    else 'vehicle')
                blueprint = str(config.get(
                    'blueprint', WALKER_BLUEPRINT
                    if actor_type == 'pedestrian' else LEAD_BLUEPRINT))
                actor = self._spawn_at(
                    blueprint, self._owned_role('actor', actor_id), transform,
                    actor_type=actor_type)
                scripted = ScriptedActor(
                    actor_id=actor_id,
                    classification=CLASSIFICATION_IDS[classification_name],
                    actor=actor,
                    config=config,
                    reference_waypoint=waypoint)
                self.scripted_actors.append(scripted)
                if config.get('legacy_role') == 'lead':
                    self.lead = actor
                elif config.get('legacy_role') == 'pedestrian':
                    self.walker = actor
        except Exception:
            self._destroy_actor_batch(
                self.spawned[initial_spawned:], flush=True)
            del self.spawned[initial_spawned:]
            del self.scripted_actors[initial_scripted:]
            self.lead = None
            self.walker = None
            raise

        if len(self.scripted_actors) != len(configs):
            raise RuntimeError('scripted actor 生成数量不完整: %d/%d' %
                               (len(self.scripted_actors), len(configs)))

    @property
    def scripted_actor_count(self):
        return len(self.scripted_actors)

    def _offset_transform_carla(self, transform, lateral_offset_carla):
        """CARLA 系横向偏移（右正）。"""
        if not lateral_offset_carla:
            return transform
        yaw = math.radians(float(transform.rotation.yaw))
        loc = self.carla.Location(
            x=float(transform.location.x - math.sin(yaw) * lateral_offset_carla),
            y=float(transform.location.y + math.cos(yaw) * lateral_offset_carla),
            z=float(transform.location.z))
        rot = self.carla.Rotation(
            pitch=float(transform.rotation.pitch),
            yaw=float(transform.rotation.yaw),
            roll=float(transform.rotation.roll))
        return self.carla.Transform(loc, rot)

    # ── 仿真推进 ──

    def tick(self):
        self.world.tick()
        t = float(self.world.get_snapshot().timestamp.elapsed_seconds)
        if self.sim_t0 is None:
            self.sim_t0 = t
        return t - self.sim_t0

    # ── 参考线推进：沿初始车道中心线按纵向投影前移 ──

    def _advance_ref(self, ego_tf):
        ego_loc = ego_tf.location
        ego_yaw = math.radians(float(ego_tf.rotation.yaw))
        wp = self._ref_wp
        for _ in range(6):
            f = wp.transform.get_forward_vector()
            dx = ego_loc.x - wp.transform.location.x
            dy = ego_loc.y - wp.transform.location.y
            s = f.x * dx + f.y * dy
            if s <= 0.3:
                break
            nxts = wp.next(min(max(s, 0.5), 5.0))
            if not nxts:
                break
            nxt = _select_forward_waypoint(
                nxts, ego_loc, ego_yaw, _waypoint_yaw(wp))
            if nxt is None:
                break
            wp = nxt
        if _xy_distance(ego_loc, wp.transform.location) > 20.0:
            nearest = self._driving_waypoint(ego_loc, project_to_road=True)
            if (nearest is not None and
                    abs(_norm_angle(_waypoint_yaw(nearest) - ego_yaw)) <=
                    REF_BRANCH_MAX_HEADING_ERROR_RAD):
                wp = nearest
        self._ref_wp = wp
        return wp

    def _ref_curvature(self, ref_wp):
        """参考线曲率：前视 CURVATURE_LOOKAHEAD_M 的 yaw 差 / 弧长，左转为正（右手系）。"""
        nxts = ref_wp.next(CURVATURE_LOOKAHEAD_M)
        if not nxts:
            return 0.0
        yaw0 = math.radians(float(ref_wp.transform.rotation.yaw))
        yaw1 = math.radians(float(nxts[0].transform.rotation.yaw))
        # CARLA 左手系 yaw 差取反 → 右手系
        return -_norm_angle(yaw1 - yaw0) / CURVATURE_LOOKAHEAD_M

    def _steering_tire_angle(self):
        """前轮转角回读 [rad]，左正。物理回读失败时回退最后指令。"""
        try:
            fl = self.ego.get_wheel_steer_angle(
                self.carla.VehicleWheelLocation.FL_Wheel)
            fr = self.ego.get_wheel_steer_angle(
                self.carla.VehicleWheelLocation.FR_Wheel)
            # CARLA 返回度、右正 → 弧度、左正
            return -math.radians(0.5 * (float(fl) + float(fr)))
        except (AttributeError, RuntimeError):
            return self._last_steer_cmd * self.max_steer_rad

    def _actor_state(self, actor, obj_id, cls):
        tf = actor.get_transform()
        yaw_c = math.radians(float(tf.rotation.yaw))
        return {
            'id': obj_id,
            'cls': cls,
            'x': float(tf.location.x),
            'y': -float(tf.location.y),
            'yaw': -yaw_c,
            'v': _speed(actor),
            'dims': self._actor_dims(actor),
        }

    def _scripted_object_states(self):
        return [
            self._actor_state(item.actor, item.actor_id, item.classification)
            for item in sorted(
                self.scripted_actors, key=lambda scripted: scripted.actor_id)
        ]

    @staticmethod
    def _actor_dims(actor):
        try:
            ext = actor.bounding_box.extent
            return (2.0 * float(ext.x), 2.0 * float(ext.y), 2.0 * float(ext.z))
        except (AttributeError, RuntimeError):
            return (4.5, 1.8, 1.5)

    # ── 感知提取：真值 → 右手系帧 ──

    def sense(self):
        ego_tf = self.ego.get_transform()
        ref_wp = self._advance_ref(ego_tf)

        yaw_c = math.radians(float(ego_tf.rotation.yaw))
        road_yaw_c = math.radians(float(ref_wp.transform.rotation.yaw))

        # 横向偏移：CARLA 系右正 = -sin*dx + cos*dy → 左正取反
        dx = ego_tf.location.x - ref_wp.transform.location.x
        dy = ego_tf.location.y - ref_wp.transform.location.y
        lat_left = -(-math.sin(road_yaw_c) * dx + math.cos(road_yaw_c) * dy)
        heading_error = _norm_angle(-(yaw_c - road_yaw_c))
        lane_valid = _lane_reference_valid(lat_left, heading_error)

        ang = self.ego.get_angular_velocity()   # deg/s，左手系

        frame = {
            'ego': {
                'x': float(ego_tf.location.x),
                'y': -float(ego_tf.location.y),
                'yaw': -yaw_c,
                'v': _speed(self.ego),
                'yaw_rate': -math.radians(float(ang.z)),
                'steer_rad': self._steering_tire_angle(),
            },
            'lane': {
                'valid': lane_valid,
                'lateral_offset': lat_left,
                'heading_error': heading_error,
                'curvature': self._ref_curvature(ref_wp) if lane_valid else 0.0,
                'lane_width': float(ref_wp.lane_width),
            },
            'objects': [],
        }
        frame['objects'] = self._scripted_object_states()
        return frame

    # ── 执行：归一化执行量 → 自车（ActuationCommand 语义）──

    def apply_ego(self, throttle, brake, steer_left):
        """throttle/brake ∈ [0,1]，steer_left ∈ [-1,1] 左正（SoC 栈约定）。"""
        throttle = _clamp(float(throttle), 0.0, 1.0)
        brake = _clamp(float(brake), 0.0, 1.0)
        steer_left = _clamp(float(steer_left), -1.0, 1.0)
        self._last_steer_cmd = steer_left
        self.ego.apply_control(self.carla.VehicleControl(
            throttle=throttle, brake=brake, steer=-steer_left))
        return throttle, brake, steer_left

    # ── 场景脚本：前车/行人驱动 ──

    def drive_actors(self, sim_t):
        for scripted in self.scripted_actors:
            if scripted.classification == CLASS_PEDESTRIAN:
                self._drive_pedestrian(scripted)
            else:
                self._drive_vehicle(scripted, sim_t)

    def _drive_vehicle(self, scripted, sim_t):
        if scripted.in_hard_brake(sim_t):
            scripted.actor.apply_control(self.carla.VehicleControl(
                throttle=0.0, brake=1.0,
                steer=self._actor_lane_keep(scripted.actor)))
            return
        v_tgt = scripted.target_speed(sim_t)
        v = _speed(scripted.actor)
        err = v_tgt - v
        scripted.velocity_integral = _clamp(
            scripted.velocity_integral + err * self.fixed_dt, -5.0, 5.0)
        a_cmd = 0.8 * err + 0.1 * scripted.velocity_integral
        throttle = _clamp(a_cmd / 3.0 + 0.05 * v / 3.0, 0.0, 0.8)
        brake = _clamp(-a_cmd / 6.0, 0.0, 1.0) if a_cmd < -0.2 else 0.0
        scripted.actor.apply_control(self.carla.VehicleControl(
            throttle=float(throttle), brake=float(brake),
            steer=self._actor_lane_keep(scripted.actor)))

    def _actor_lane_keep(self, actor):
        """目标车简易车道保持（P 控制航向 + 横向偏差）。"""
        tf = actor.get_transform()
        wp = self.map.get_waypoint(tf.location, project_to_road=True,
                                   lane_type=self.carla.LaneType.Driving)
        if wp is None:
            return 0.0
        nxt = wp.next(6.0)
        if not nxt:
            return 0.0
        tgt = nxt[0].transform
        yaw = math.radians(float(tf.rotation.yaw))
        tgt_yaw = math.radians(float(tgt.rotation.yaw))
        he = _norm_angle(tgt_yaw - yaw)
        dx = tf.location.x - wp.transform.location.x
        dy = tf.location.y - wp.transform.location.y
        ryaw = math.radians(float(wp.transform.rotation.yaw))
        lat = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        return _clamp(1.2 * he - 0.15 * lat, -0.5, 0.5)

    def _drive_pedestrian(self, scripted):
        """自车逼近到触发距离后，行人垂直横穿车道（AEB 场景）。

        坐标系约定（与 scenario JSON / sim_vehicle_core 对齐）：
          * JSON ``initial_lateral_m`` / ``end_lateral_m``：左正、右负（右手系）
          * CARLA 自车坐标系：x 前、y 右（UE4 左手系）
          * 车道左法向（CARLA 世界系）：( sin(yaw), -cos(yaw) )
            + yaw=0 时 = (0, -1) = CARLA -y = 车体左侧 ✓
          * spawn 时 lateral_left 取反传给 _offset_transform_carla（CARLA 右正）
            + JSON lateral=-6（右路肩）→ CARLA 偏移 +6 = +y = 右侧 ✓
          * 本函数：direction=+1 → 沿车道左法向走 = 朝自车所在侧横穿

        方向判定（修复）：JSON 起点 lateral 与终点 lateral 谁更大决定 direction。
        end_lateral_m 默认 ``-initial_lateral_m``（对称穿越到对侧车道），
        与 SIL overlay（scenario_overlay.py:153-154）一致。
        """
        config = scripted.config
        ref_tf = scripted.reference_waypoint.transform
        yaw = math.radians(float(ref_tf.rotation.yaw))

        # ── 1) 触发检测 ──────────────────────────────────────────
        if not scripted.triggered:
            gap = _xy_distance(self.ego.get_transform().location,
                               scripted.actor.get_transform().location)
            if gap > float(config.get('trigger_ego_gap_m', 35.0)):
                return
            scripted.triggered = True

        initial_lateral_m = float(config.get('initial_lateral_m', -6.0))
        end_lateral_m = config.get('end_lateral_m')
        if end_lateral_m is None:
            end_lateral_m = -initial_lateral_m   # 默认对称穿越
        else:
            end_lateral_m = float(end_lateral_m)

        # ── 2) 终点检测（actor 已在 JSON 横向系下越过 end_lateral_m） ──
        actor_loc = scripted.actor.get_transform().location
        # JSON 左正 ↔ CARLA 偏移符号相反：lateral = +dx*sin(yaw) - dy*cos(yaw)
        current_lateral = (
            (float(actor_loc.x) - float(ref_tf.location.x)) * math.sin(yaw)
            - (float(actor_loc.y) - float(ref_tf.location.y)) * math.cos(yaw))
        direction_sign = (
            1.0 if end_lateral_m > initial_lateral_m else -1.0)
        traveled = (current_lateral - initial_lateral_m) * direction_sign
        total_distance = abs(end_lateral_m - initial_lateral_m)

        if scripted.triggered and total_distance > 1e-3 and traveled >= total_distance:
            # 已到达或越过对侧车道
            scripted.actor.apply_control(self.carla.WalkerControl(
                direction=self.carla.Vector3D(x=0.0, y=0.0, z=0.0),
                speed=0.0, jump=False))
            return

        # ── 3) 横穿运动：沿车道左法向（CARLA 系） ──────────────────
        # direction_sign = +1 → velocity 沿 (sin(yaw), -cos(yaw)) = 左法向
        vx = math.sin(yaw) * direction_sign
        vy = -math.cos(yaw) * direction_sign
        scripted.actor.apply_control(self.carla.WalkerControl(
            direction=self.carla.Vector3D(x=float(vx), y=float(vy), z=0.0),
            speed=float(config.get('crossing_speed_mps', 1.5)),
            jump=False))

    # ── 旁观者跟车视角 ──

    def update_spectator(self):
        tf = self.ego.get_transform()
        yaw = math.radians(float(tf.rotation.yaw))
        cam_loc = self.carla.Location(
            x=tf.location.x - 12.0 * math.cos(yaw),
            y=tf.location.y - 12.0 * math.sin(yaw),
            z=tf.location.z + 6.0)
        cam_rot = self.carla.Rotation(
            pitch=-18.0, yaw=float(tf.rotation.yaw), roll=0.0)
        try:
            self.world.get_spectator().set_transform(
                self.carla.Transform(cam_loc, cam_rot))
        except RuntimeError:
            pass

    # ── CARLA 世界内 ADAS 展示（GUI 仅负责控制）──

    @staticmethod
    def _behavior_name(state):
        return {
            0: 'LANE FOLLOW', 1: 'FOLLOW LEAD', 2: 'OVERTAKE WAIT',
            3: 'OVERTAKING', 4: 'RETURN LANE', 5: 'STOPPING',
            6: 'EMERGENCY', 7: 'APPROACH STOP', 8: 'STOPPED',
            9: 'WAIT LIGHT', 10: 'JUNCTION',
        }.get(int(state), 'WAITING')

    @staticmethod
    def _nav_name(state):
        return {
            0: 'IDLE', 1: 'WAIT MAP', 2: 'PLANNING', 3: 'DRIVING',
            4: 'ARRIVED', 5: 'FAILED', 6: 'CANCELED',
        }.get(int(state), 'UNKNOWN')

    def _debug_location(self, ros_x, ros_y, z=0.35):
        """ROS/ADAS 右手系 (x,y) → CARLA 左手系 (x,-y)。"""
        return self.carla.Location(x=float(ros_x), y=-float(ros_y), z=float(z))

    def draw_adas_visualization(self, state, frame, actuation, sim_t):
        """在 CARLA 世界绘制路线、终点、目标标签和驾驶状态。

        所有元素使用短 lifetime 并周期刷新，路线取消或场景退出后会自然消失，
        不会把旧一次运行的 debug primitive 留在世界中。
        """
        if self.no_rendering or sim_t - self._last_visualization_t < 0.20:
            return
        self._last_visualization_t = sim_t
        debug = self.world.debug
        life = 0.28
        blue = self.carla.Color(40, 150, 255)
        green = self.carla.Color(40, 230, 110)
        amber = self.carla.Color(255, 180, 30)
        red = self.carla.Color(255, 45, 45)
        white = self.carla.Color(245, 245, 245)

        route = state.get('route', [])
        pending_goal = state.get('pending_goal')
        if len(route) >= 2:
            points = [self._debug_location(x, y) for x, y in route]
            for start, end in zip(points, points[1:]):
                debug.draw_line(start, end, thickness=0.16, color=blue,
                                life_time=life, persistent_lines=False)
            goal = points[-1]
            debug.draw_point(goal, size=0.28, color=green,
                             life_time=life, persistent_lines=False)
            debug.draw_string(
                self.carla.Location(x=goal.x, y=goal.y, z=goal.z + 1.0),
                'NAV GOAL', draw_shadow=True, color=green,
                life_time=life, persistent_lines=False)
        elif pending_goal is not None:
            # 服务已受理、路线尚在规划时也立即在 CARLA 中显示目标。
            goal = self._debug_location(pending_goal[0], pending_goal[1])
            debug.draw_point(goal, size=0.28, color=green,
                             life_time=life, persistent_lines=False)
            debug.draw_string(
                self.carla.Location(x=goal.x, y=goal.y, z=goal.z + 1.0),
                'GOAL - PLANNING', draw_shadow=True, color=green,
                life_time=life, persistent_lines=False)

        # 目标物本身由 CARLA actor 展示；这里只叠加稳定感知 ID/类别，证明
        # 感知结果与仿真 actor 对得上，而不是在 GUI 中另画一套替身。
        class_names = {1: 'CAR', 2: 'TRUCK', 3: 'PEDESTRIAN', 4: 'BICYCLE'}
        for scripted in self.scripted_actors:
            try:
                loc = scripted.actor.get_location()
                label_loc = self.carla.Location(
                    x=loc.x, y=loc.y, z=loc.z +
                    (2.4 if scripted.classification != CLASS_PEDESTRIAN else 1.8))
                debug.draw_string(
                    label_loc,
                    'OBJ %d  %s' % (scripted.actor_id,
                                     class_names.get(scripted.classification,
                                                     'UNKNOWN')),
                    draw_shadow=True, color=amber, life_time=life,
                    persistent_lines=False)
            except RuntimeError:
                continue

        ego_tf = self.ego.get_transform()
        ego_label = self.carla.Location(
            x=ego_tf.location.x, y=ego_tf.location.y, z=ego_tf.location.z + 2.8)
        speed_kph = max(0.0, float(frame['ego']['v'])) * 3.6
        target = float(state.get('target_speed_mps', float('nan')))
        remaining = float(state.get('remaining_m', float('nan')))
        line1 = 'EGO  %4.1f km/h  |  %s' % (
            speed_kph, self._behavior_name(state.get('behavior_state', -1)))
        if math.isfinite(target):
            line1 += '  TARGET %.0f' % (max(0.0, target) * 3.6)
        line2 = 'NAV %s' % self._nav_name(state.get('nav_state', 0))
        if math.isfinite(remaining) and remaining >= 0.0:
            line2 += '  %.0f m' % remaining
        line2 += '  |  %s' % ('CONTROL STALE' if actuation.get('stale') else
                              'CONTROL OK')
        aeb_state = int(state.get('aeb_state', 0))
        safety_level = int(state.get('safety_level', 0))
        status_color = red if aeb_state >= 3 or safety_level >= 2 else (
            amber if aeb_state == 2 or safety_level == 1 else white)
        debug.draw_string(ego_label, line1, draw_shadow=True, color=white,
                          life_time=life, persistent_lines=False)
        debug.draw_string(
            self.carla.Location(x=ego_label.x, y=ego_label.y,
                                z=ego_label.z + 0.45),
            line2, draw_shadow=True, color=status_color,
            life_time=life, persistent_lines=False)
        if aeb_state >= 2:
            ttc = float(state.get('ttc_s', float('nan')))
            aeb_text = 'AEB EMERGENCY' if aeb_state >= 3 else 'AEB WARNING'
            if math.isfinite(ttc) and ttc < 1.0e5:
                aeb_text += '  TTC %.1fs' % ttc
            debug.draw_string(
                self.carla.Location(x=ego_label.x, y=ego_label.y,
                                    z=ego_label.z + 0.9),
                aeb_text, draw_shadow=True, color=red if aeb_state >= 3 else amber,
                life_time=life, persistent_lines=False)

    def close(self):
        if self.ego is not None:
            try:
                self.ego.apply_control(self.carla.VehicleControl(brake=1.0))
            except RuntimeError:
                pass
        # P0.D: 先清 debug overlay，再清理本桥创建的 actor；最后还原设置。
        # 多次调用必须安全。
        self.clear_overlays()
        cleanup_error = None
        try:
            self._destroy_spawned(flush=True)
        except RuntimeError as error:
            cleanup_error = error
        finally:
            try:
                self.world.apply_settings(self.original_settings)
            except RuntimeError:
                pass
        if cleanup_error is not None:
            raise cleanup_error

    def clear_overlays(self):
        """P0.D: 幂等清空所有由本桥绘制的 debug primitive。

        CARLA 没有按"actor 全部销毁"的清屏 API，调用 world.debug.clear() 才
        能一次性清掉所有持久调试元素。bridge 停止或 session 结束时必须调用，
        避免旧会话的 goal/route 残留在 CARLA 视图中。"""
        try:
            if getattr(self, 'world', None) is not None:
                self.world.debug.clear()
        except RuntimeError:
            # 已销毁或无渲染时静默；幂等是本方法的硬性约束。
            pass
        finally:
            self._last_visualization_t = float('-inf')
