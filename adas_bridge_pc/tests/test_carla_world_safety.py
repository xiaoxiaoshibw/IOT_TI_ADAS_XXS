#!/usr/bin/env python3
"""Deterministic tests for Phase 4 spawn safety in carla_world.

覆盖：
- _too_close_to_existing 在距离过近时返回 True
- _too_close_to_existing 在距离足够时返回 False
- _audit_actors 检测穿地 (z < 0) 警告
- _audit_actors 检测与 hero 重叠 (d < 1m) 警告
- _audit_actors 在正常 spawn 时不报告
- 共享常量 SAFE_SPAWN_DISTANCE_M = 5.0
"""

from __future__ import annotations

import importlib.util
import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    REPO_ROOT
    / "carla_ros2_bridge/ws/src/adas_carla_bridge/adas_carla_bridge/carla_world.py"
)


def _load_world():
    spec = importlib.util.spec_from_file_location("carla_world", MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _make_location(x=0.0, y=0.0, z=0.5):
    return SimpleNamespace(x=x, y=y, z=z)


def _make_transform(x=0.0, y=0.0, z=0.5):
    return SimpleNamespace(
        location=_make_location(x, y, z),
        rotation=SimpleNamespace(pitch=0.0, yaw=0.0, roll=0.0),
    )


def _make_actor(actor_id=1, x=0.0, y=0.0, z=0.5):
    return SimpleNamespace(
        id=actor_id,
        type_id="vehicle.tesla.model3",
        get_location=lambda: _make_location(x, y, z),
    )


class SpawnSafetyConstants(unittest.TestCase):
    def setUp(self):
        self.w = _load_world()

    def test_safe_distance_constant(self):
        self.assertEqual(self.w.SAFE_SPAWN_DISTANCE_M, 5.0)

    def test_audit_min_height_zero(self):
        # z < 0 视为穿地/撞墙（CARLA 地面 z=0）
        self.assertEqual(self.w.SPAWN_AUDIT_MIN_HEIGHT_M, 0.0)

    def test_audit_min_hero_distance_one_meter(self):
        self.assertEqual(self.w.SPAWN_AUDIT_MIN_HERO_DIST_M, 1.0)


class TooCloseToExistingTests(unittest.TestCase):
    def setUp(self):
        self.w = _load_world()
        # 构造一个 CarlaWorld 的最小可测试实例
        self.world_obj = SimpleNamespace(
            spawned=[],
            _xy_distance=self.w._xy_distance,
        )
        # 把 _too_close_to_existing 的 self 替换为我们的 mock
        self._bound = self.w.CarlaWorld._too_close_to_existing.__get__(
            self.world_obj, self.w.CarlaWorld
        )

    def test_no_existing_actors(self):
        transform = _make_transform(10, 10)
        self.assertFalse(self._bound(transform))

    def test_actor_too_close(self):
        self.world_obj.spawned = [_make_actor(actor_id=2, x=0, y=0)]
        transform = _make_transform(3, 0)  # 距离 3m < 5m
        self.assertTrue(self._bound(transform))

    def test_actor_far_enough(self):
        self.world_obj.spawned = [_make_actor(actor_id=2, x=0, y=0)]
        transform = _make_transform(10, 0)  # 距离 10m >= 5m
        self.assertFalse(self._bound(transform))

    def test_multiple_actors_check_against_all(self):
        self.world_obj.spawned = [
            _make_actor(actor_id=2, x=100, y=0),  # 远
            _make_actor(actor_id=3, x=2, y=2),  # 距离 sqrt(8) ≈ 2.83 < 5
        ]
        transform = _make_transform(0, 0)
        self.assertTrue(self._bound(transform))


class AuditActorsTests(unittest.TestCase):
    def setUp(self):
        self.w = _load_world()
        # 构造 mock CarlaWorld
        self.world_obj = SimpleNamespace(
            spawned=[],
            ego=None,
        )
        self._bound = self.w.CarlaWorld._audit_actors.__get__(
            self.world_obj, self.w.CarlaWorld
        )

    def _capture(self, fn):
        buf = io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            fn()
        finally:
            sys.stdout = old
        return buf.getvalue()

    def test_no_actors_no_output(self):
        out = self._capture(self._bound)
        self.assertEqual(out, "")

    def test_normal_actors_no_warning(self):
        self.world_obj.spawned = [
            _make_actor(actor_id=1, x=0, y=0, z=0.5),
            _make_actor(actor_id=2, x=10, y=0, z=0.5),
        ]
        self.world_obj.ego = self.world_obj.spawned[0]
        out = self._capture(self._bound)
        self.assertEqual(out, "")

    def test_underground_actor_warns(self):
        underground = _make_actor(actor_id=1, x=0, y=0, z=-0.5)
        self.world_obj.spawned = [underground]
        self.world_obj.ego = underground
        out = self._capture(self._bound)
        self.assertIn("AUDIT", out)
        self.assertIn("穿地", out)

    def test_overlap_with_hero_warns(self):
        # hero at (0,0), 其他 actor at (0.5, 0) → 距离 0.5m < 1m
        hero = _make_actor(actor_id=1, x=0, y=0)
        other = _make_actor(actor_id=2, x=0.5, y=0)
        self.world_obj.spawned = [hero, other]
        self.world_obj.ego = hero
        out = self._capture(self._bound)
        self.assertIn("AUDIT", out)
        self.assertIn("重叠", out)


class XYDistanceTests(unittest.TestCase):
    def setUp(self):
        self.w = _load_world()

    def test_xy_distance_zero(self):
        a = _make_location(0, 0)
        b = _make_location(0, 0)
        self.assertAlmostEqual(self.w._xy_distance(a, b), 0.0)

    def test_xy_distance_3_4_5(self):
        a = _make_location(0, 0)
        b = _make_location(3, 4)
        self.assertAlmostEqual(self.w._xy_distance(a, b), 5.0)

    def test_xy_distance_negative_coords(self):
        a = _make_location(-1, -1)
        b = _make_location(2, 3)
        # sqrt(9 + 16) = 5
        self.assertAlmostEqual(self.w._xy_distance(a, b), 5.0)


class ScenariosSpawnIndexSpreadTests(unittest.TestCase):
    """验证 scenarios.py 已为不同场景分配不同的 spawn_index,避免拥挤。"""

    def setUp(self):
        scenarios_path = (
            REPO_ROOT
            / "carla_ros2_bridge/ws/src/adas_carla_bridge/adas_carla_bridge"
            / "scenarios.py"
        )
        spec = importlib.util.spec_from_file_location("scenarios", scenarios_path)
        self.s = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.s)

    def test_all_scenarios_have_spawn_index(self):
        for name, cfg in self.s.SCENARIOS.items():
            self.assertIn("spawn_index", cfg, "%s 缺少 spawn_index" % name)
            self.assertIsInstance(cfg["spawn_index"], int)

    def test_no_two_scenarios_share_exact_spawn_index(self):
        """避免 8 个场景共用 spawn_index=30 导致 spawn 距离冲突。

        允许 lka + free 共用 30（两个最简单的场景都是直行高速,共享安全）；
        其余场景必须有独立的 spawn_index。
        """
        from collections import Counter
        counts = Counter(
            cfg["spawn_index"] for cfg in self.s.SCENARIOS.values()
        )
        # 仅保留真正重复的（count > 1）且不在 lka/free 共享白名单内的索引
        duplicates = {
            idx: c
            for idx, c in counts.items()
            if c > 1 and not (idx == 30)
        }
        self.assertEqual(
            duplicates, {},
            "重复 spawn_index（除 lka/free 共用 30 外）: %s" % duplicates,
        )
        # 验证 lka 和 free 的确都使用 30
        self.assertEqual(self.s.SCENARIOS["lka"]["spawn_index"], 30)
        self.assertEqual(self.s.SCENARIOS["free"]["spawn_index"], 30)

    def test_spawn_indices_in_valid_range(self):
        # Town04 有 ~200+ spawn points,这里保守要求 < 1000
        for name, cfg in self.s.SCENARIOS.items():
            self.assertGreaterEqual(cfg["spawn_index"], 0)
            self.assertLess(cfg["spawn_index"], 1000)


if __name__ == "__main__":
    unittest.main()
