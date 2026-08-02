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

CARLA_TIMEOUT_S = 60.0
SPAWN_WAYPOINT_MAX_DIST_M = 4.0
EGO_BLUEPRINT = 'vehicle.tesla.model3'
LEAD_BLUEPRINT = 'vehicle.audi.tt'
WALKER_BLUEPRINT = 'walker.pedestrian.*'

# adas_msgs/TrackedObject 分类常量（与 msg 定义一致，避免此处依赖 ROS）
CLASS_CAR = 1
CLASS_PEDESTRIAN = 3

# 曲率估计的前视弧长 [m]
CURVATURE_LOOKAHEAD_M = 8.0


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


class CarlaWorld:
    """持有 CARLA 客户端与场景演员，输出右手系真值帧、接收归一化执行量。"""

    def __init__(self, carla, host, port, scenario, town='Town04',
                 fixed_dt=0.02, no_rendering=False, spawn_index=None):
        self.carla = carla
        self.scenario = scenario
        self.fixed_dt = float(fixed_dt)

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
            spawn_index = int(scenario.get('spawn_index', 30))

        self.spawned = []
        self._cleanup_previous_actors()
        self.ego = self._spawn(EGO_BLUEPRINT, 'hero', spawn_index)
        self.world.tick()

        # ── 前车（可选）──
        self.lead = None
        self._lead_cfg = scenario.get('lead') or {}
        self._lead_v_int = 0.0
        if self._lead_cfg:
            ego_wp = self.map.get_waypoint(self.ego.get_transform().location)
            gap0 = float(self._lead_cfg.get('gap0', 40.0))
            lead_wps = ego_wp.next(gap0)
            if not lead_wps:
                raise RuntimeError('无法在前方 %.0fm 找到前车生成点' % gap0)
            bp = str(self._lead_cfg.get('blueprint', LEAD_BLUEPRINT))
            self.lead = self._spawn_at(bp, 'lead', lead_wps[0].transform)

        # ── 横穿行人（可选，AEB 场景）──
        self.walker = None
        self._walker_cfg = scenario.get('pedestrian') or {}
        self._walker_triggered = False
        if self._walker_cfg:
            ego_wp = self.map.get_waypoint(self.ego.get_transform().location)
            ahead = float(self._walker_cfg.get('ahead_m', 80.0))
            wps = ego_wp.next(ahead)
            if not wps:
                raise RuntimeError('无法在前方 %.0fm 找到行人生成点' % ahead)
            # start_lateral_m 为右手系左正 → CARLA 侧偏移取反
            lat_r = float(self._walker_cfg.get('start_lateral_m', -6.0))
            tf = self._offset_transform_carla(wps[0].transform, -lat_r)
            self.walker = self._spawn_at(WALKER_BLUEPRINT, 'lead', tf,
                                         actor_type='pedestrian')
            self._walker_wp = wps[0]

        phys = self.ego.get_physics_control()
        self.max_steer_rad = math.radians(
            max(w.max_steer_angle for w in phys.wheels) or 70.0)

        self.sim_t0 = None
        self._last_steer_cmd = 0.0    # 左正（右手系），转角回读兜底用

        # 参考中心线：锚定自车初始车道
        self._ref_wp = self._driving_waypoint(
            self.ego.get_transform().location, project_to_road=False)
        if self._ref_wp is None:
            self._ref_wp = self._driving_waypoint(
                self.ego.get_transform().location, project_to_road=True)
        if self._ref_wp is None:
            raise RuntimeError('自车不在可驾驶车道附近，无法初始化参考线')

        for _ in range(5):
            self.world.tick()

    # ── 生成/清理 ──

    def _driving_waypoint(self, location, project_to_road):
        try:
            return self.map.get_waypoint(
                location, project_to_road=project_to_road,
                lane_type=self.carla.LaneType.Driving)
        except RuntimeError:
            return None

    def _cleanup_previous_actors(self):
        stale = []
        for pattern in ('vehicle.*', 'walker.pedestrian.*'):
            for actor in self.world.get_actors().filter(pattern):
                if actor.attributes.get('role_name', '') in ('hero', 'lead'):
                    stale.append(actor)
        if stale:
            print('cleaning %d stale actor(s)' % len(stale), flush=True)
            for actor in stale:
                try:
                    actor.destroy()
                except RuntimeError:
                    pass
            for _ in range(2):
                self.world.tick()

    def _aligned_spawn_transform(self, transform):
        wp = self._driving_waypoint(transform.location, project_to_road=False)
        if wp is None:
            wp = self._driving_waypoint(transform.location, project_to_road=True)
        if wp is None:
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
        transform.location.z += 0.2 if actor_type == 'pedestrian' else 0.5
        actor = self.world.try_spawn_actor(bp, transform)
        if actor is None:
            raise RuntimeError('无法生成目标 %s' % bp_name)
        self.spawned.append(actor)
        return actor

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

    def _advance_ref(self, ego_loc):
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
            wp = min(nxts, key=lambda w:
                     (w.transform.location.x - ego_loc.x) ** 2
                     + (w.transform.location.y - ego_loc.y) ** 2)
        if _xy_distance(ego_loc, wp.transform.location) > 20.0:
            nearest = self._driving_waypoint(ego_loc, project_to_road=True)
            if nearest is not None:
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
        ref_wp = self._advance_ref(ego_tf.location)

        yaw_c = math.radians(float(ego_tf.rotation.yaw))
        road_yaw_c = math.radians(float(ref_wp.transform.rotation.yaw))

        # 横向偏移：CARLA 系右正 = -sin*dx + cos*dy → 左正取反
        dx = ego_tf.location.x - ref_wp.transform.location.x
        dy = ego_tf.location.y - ref_wp.transform.location.y
        lat_left = -(-math.sin(road_yaw_c) * dx + math.cos(road_yaw_c) * dy)

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
                'valid': True,
                'lateral_offset': lat_left,
                'heading_error': _norm_angle(-(yaw_c - road_yaw_c)),
                'curvature': self._ref_curvature(ref_wp),
                'lane_width': float(ref_wp.lane_width),
            },
            'objects': [],
        }
        if self.lead is not None:
            frame['objects'].append(self._actor_state(self.lead, 1, CLASS_CAR))
        if self.walker is not None:
            frame['objects'].append(
                self._actor_state(self.walker, 2, CLASS_PEDESTRIAN))
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
        if self.lead is not None:
            self._drive_lead(sim_t)
        if self.walker is not None:
            self._drive_walker()

    def _lead_target_speed(self, sim_t):
        v = 0.0
        for t, spd in self._lead_cfg.get('profile', [(0.0, 0.0)]):
            if sim_t >= float(t):
                v = float(spd)
        return v

    def _lead_in_hard_brake(self, sim_t):
        hb = self._lead_cfg.get('hard_brake')
        return bool(hb) and float(hb[0]) <= sim_t <= float(hb[1])

    def _drive_lead(self, sim_t):
        if self._lead_in_hard_brake(sim_t):
            self.lead.apply_control(self.carla.VehicleControl(
                throttle=0.0, brake=1.0, steer=self._lead_lane_keep()))
            return
        v_tgt = self._lead_target_speed(sim_t)
        v = _speed(self.lead)
        err = v_tgt - v
        self._lead_v_int = _clamp(self._lead_v_int + err * self.fixed_dt,
                                  -5.0, 5.0)
        a_cmd = 0.8 * err + 0.1 * self._lead_v_int
        throttle = _clamp(a_cmd / 3.0 + 0.05 * v / 3.0, 0.0, 0.8)
        brake = _clamp(-a_cmd / 6.0, 0.0, 1.0) if a_cmd < -0.2 else 0.0
        self.lead.apply_control(self.carla.VehicleControl(
            throttle=float(throttle), brake=float(brake),
            steer=self._lead_lane_keep()))

    def _lead_lane_keep(self):
        """前车简易车道保持（P 控制航向 + 横向偏差，CARLA 系内自洽）。"""
        tf = self.lead.get_transform()
        wp = self.map.get_waypoint(tf.location, project_to_road=True,
                                   lane_type=self.carla.LaneType.Driving)
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

    def _drive_walker(self):
        """自车逼近到触发距离后，行人垂直横穿车道（AEB 场景）。"""
        if not self._walker_triggered:
            gap = _xy_distance(self.ego.get_transform().location,
                               self.walker.get_transform().location)
            if gap > float(self._walker_cfg.get('trigger_ego_gap_m', 35.0)):
                return
            self._walker_triggered = True
        yaw = math.radians(float(self._walker_wp.transform.rotation.yaw))
        # start_lateral_m 左正（右手系）：从左侧出发向右横穿，反之向左
        direction = -1.0 if float(
            self._walker_cfg.get('start_lateral_m', -6.0)) < 0.0 else 1.0
        # CARLA 系：车道左法向 = (sin(yaw), -cos(yaw))
        vx = math.sin(yaw) * direction
        vy = -math.cos(yaw) * direction
        self.walker.apply_control(self.carla.WalkerControl(
            direction=self.carla.Vector3D(x=float(vx), y=float(vy), z=0.0),
            speed=float(self._walker_cfg.get('speed_mps', 1.5)),
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

    def close(self):
        try:
            self.ego.apply_control(self.carla.VehicleControl(brake=1.0))
        except RuntimeError:
            pass
        for actor in self.spawned:
            try:
                actor.destroy()
            except RuntimeError:
                pass
        try:
            self.world.apply_settings(self.original_settings)
        except RuntimeError:
            pass
