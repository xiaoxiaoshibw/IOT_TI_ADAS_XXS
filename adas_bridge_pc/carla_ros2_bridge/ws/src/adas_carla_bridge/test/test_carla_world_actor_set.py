import math
from types import SimpleNamespace

import pytest

from adas_carla_bridge.carla_world import (
    CLASS_CAR,
    CarlaWorld,
    ScriptedActor,
    _lane_change_available,
)


def actor_config(actor_id, station=20.0):
    return {
        "id": actor_id,
        "classification": "vehicle",
        "lane": "ego",
        "initial_station_m": station,
        "initial_lateral_m": 0.0,
        "initial_speed_mps": 4.0,
        "accel_limit_mps2": 3.0,
        "speed_profile": [[0.0, 4.0], [10.0, 7.0]],
    }


class FakeActor:
    def __init__(self, x=0.0, actor_id=None):
        if actor_id is not None:
            self.id = actor_id
        self.destroyed = False
        self.is_alive = True
        self.transform = SimpleNamespace(
            location=SimpleNamespace(x=x, y=0.0, z=0.0),
            rotation=SimpleNamespace(yaw=0.0),
        )
        self.bounding_box = SimpleNamespace(
            extent=SimpleNamespace(x=2.25, y=0.9, z=0.75))

    def destroy(self):
        self.destroyed = True
        self.is_alive = False

    def get_transform(self):
        return self.transform

    def get_velocity(self):
        return SimpleNamespace(x=4.0, y=0.0, z=0.0)


def fake_world(configs):
    world = CarlaWorld.__new__(CarlaWorld)
    world.scenario = {"actors": configs}
    world.spawned = []
    world.scripted_actors = []
    world.lead = None
    world.walker = None
    world.run_marker = "test-run"
    waypoint = SimpleNamespace(transform=SimpleNamespace())
    world._waypoint_at_station = lambda _origin, _station: waypoint
    world._offset_transform_carla = lambda transform, _offset: transform
    return world, waypoint


def test_profile_boundaries_are_deterministic():
    scripted = ScriptedActor(
        actor_id=7,
        classification=CLASS_CAR,
        actor=FakeActor(),
        config=actor_config(7),
        reference_waypoint=None,
    )

    assert scripted.target_speed(0.0) == 4.0
    assert scripted.target_speed(9.999) == 4.0
    assert scripted.target_speed(10.0) == 7.0


def test_actor_state_uses_actual_velocity_heading_for_crossing_walker():
    actor = FakeActor(x=30.0)
    actor.transform.location.y = 4.0
    actor.transform.rotation.yaw = 0.0
    actor.get_velocity = lambda: SimpleNamespace(x=0.0, y=-1.5, z=0.0)
    world = CarlaWorld.__new__(CarlaWorld)

    state = world._actor_state(actor, 2, 3)

    assert state["x"] == pytest.approx(30.0)
    assert state["y"] == pytest.approx(-4.0)
    assert state["v"] == pytest.approx(1.5)
    assert state["yaw"] == pytest.approx(math.pi / 2.0)


def test_leftmost_lane_reports_no_left_lane_change():
    leftmost = SimpleNamespace(
        lane_change='Left', lane_id=-2,
        get_left_lane=lambda: None)
    assert not _lane_change_available(leftmost, 'left')


def test_left_lane_requires_permission_driving_and_same_direction():
    adjacent = SimpleNamespace(lane_type='Driving', lane_id=-2)
    current = SimpleNamespace(
        lane_change='Both', lane_id=-1,
        get_left_lane=lambda: adjacent,
        get_right_lane=lambda: None)
    assert _lane_change_available(current, 'left')

    current.lane_change = 'Right'
    assert not _lane_change_available(current, 'left')
    current.lane_change = 'Both'
    adjacent.lane_type = 'Shoulder'
    assert not _lane_change_available(current, 'left')
    adjacent.lane_type = 'Driving'
    adjacent.lane_id = 1
    assert not _lane_change_available(current, 'left')


def test_station_walk_uses_local_heading_across_long_curve():
    class CurvedWaypoint:
        def __init__(self, station, yaw):
            self.station = station
            self.transform = SimpleNamespace(
                location=SimpleNamespace(x=station, y=0.0),
                rotation=SimpleNamespace(yaw=yaw),
            )

        def next(self, distance):
            return [CurvedWaypoint(
                self.station + distance, self.transform.rotation.yaw + 5.0)]

        def previous(self, distance):
            return [CurvedWaypoint(
                self.station - distance, self.transform.rotation.yaw - 5.0)]

    world = CarlaWorld.__new__(CarlaWorld)
    result = world._waypoint_at_station(CurvedWaypoint(0.0, 0.0), 120.0)

    assert result.station == pytest.approx(120.0)
    assert result.transform.rotation.yaw > 90.0


def test_spawn_is_id_ordered_and_roles_are_run_scoped():
    world, waypoint = fake_world([actor_config(3), actor_config(9, 40.0)])
    roles = []

    def spawn(_blueprint, role, _transform, actor_type="vehicle"):
        assert actor_type == "vehicle"
        actor = FakeActor()
        roles.append(role)
        world.spawned.append(actor)
        return actor

    world._spawn_at = spawn
    world._spawn_scripted_actors(waypoint)

    assert [actor.actor_id for actor in world.scripted_actors] == [3, 9]
    assert roles == ["adas:test-run:actor:3", "adas:test-run:actor:9"]
    assert world.scripted_actor_count == 2


def test_partial_spawn_failure_rolls_back_only_this_actor_set():
    world, waypoint = fake_world([actor_config(1), actor_config(2, 40.0)])
    external = FakeActor()
    first_owned = FakeActor()

    def spawn(_blueprint, role, _transform, actor_type="vehicle"):
        assert actor_type == "vehicle"
        if role.endswith(":2"):
            raise RuntimeError("intentional spawn failure")
        world.spawned.append(first_owned)
        return first_owned

    world._spawn_at = spawn
    with pytest.raises(RuntimeError, match="intentional spawn failure"):
        world._spawn_scripted_actors(waypoint)

    assert first_owned.destroyed
    assert not external.destroyed
    assert world.spawned == []
    assert world.scripted_actors == []


def test_destroy_spawned_does_not_scan_or_destroy_external_actors():
    world, _ = fake_world([])
    owned = FakeActor()
    external = FakeActor()
    world.spawned = [owned]

    world._destroy_spawned()

    assert owned.destroyed
    assert not external.destroyed
    assert world.spawned == []


def test_destroy_spawned_ticks_same_world_and_confirms_delayed_removal():
    class DelayedActor(FakeActor):
        def destroy(self):
            self.destroyed = True
            self.pending_destroy = True

    class FakeServerWorld:
        def __init__(self, actors):
            self.actors = actors
            self.tick_count = 0

        def tick(self):
            self.tick_count += 1
            for actor in self.actors:
                if getattr(actor, "pending_destroy", False):
                    actor.is_alive = False
            self.actors = [actor for actor in self.actors if actor.is_alive]

        def get_actors(self):
            return list(self.actors)

    world, _ = fake_world([])
    owned = DelayedActor(actor_id=42)
    external = FakeActor(actor_id=99)
    world.world = FakeServerWorld([owned, external])
    world.spawned = [owned]

    world._destroy_spawned(flush=True)

    assert owned.destroyed
    assert world.world.tick_count == 1
    assert [actor.id for actor in world.world.get_actors()] == [99]


def test_clear_overlays_uses_carla_0916_split_clear_api():
    calls = []
    debug = SimpleNamespace(
        clear_debug_shape=lambda: calls.append("shape"),
        clear_debug_string=lambda: calls.append("string"),
    )
    world = CarlaWorld.__new__(CarlaWorld)
    world.world = SimpleNamespace(debug=debug)
    world._last_visualization_t = 12.0

    world.clear_overlays()

    assert calls == ["shape", "string"]
    assert world._last_visualization_t == float("-inf")


def test_clear_overlays_falls_back_to_legacy_clear_and_tolerates_no_api():
    calls = []
    world = CarlaWorld.__new__(CarlaWorld)
    world.world = SimpleNamespace(
        debug=SimpleNamespace(clear=lambda: calls.append("legacy")))
    world._last_visualization_t = 12.0
    world.clear_overlays()
    assert calls == ["legacy"]

    world.world = SimpleNamespace(debug=SimpleNamespace())
    world._last_visualization_t = 12.0
    world.clear_overlays()
    assert world._last_visualization_t == float("-inf")


def test_object_states_are_sorted_by_stable_id():
    world, _ = fake_world([])
    world.scripted_actors = [
        ScriptedActor(20, CLASS_CAR, FakeActor(20.0), actor_config(20), None),
        ScriptedActor(4, CLASS_CAR, FakeActor(4.0), actor_config(4), None),
    ]

    objects = world._scripted_object_states()

    assert [obj["id"] for obj in objects] == [4, 20]
    assert all(obj["cls"] == CLASS_CAR for obj in objects)


def test_carla_visualization_draws_ros_route_with_y_axis_conversion():
    class Location:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    class Color:
        def __init__(self, red, green, blue):
            self.rgb = (red, green, blue)

    class Debug:
        def __init__(self):
            self.lines = []
            self.points = []
            self.strings = []

        def draw_line(self, start, end, **_kwargs):
            self.lines.append((start, end))

        def draw_point(self, point, **_kwargs):
            self.points.append(point)

        def draw_string(self, point, value, **_kwargs):
            self.strings.append((point, value))

    world = CarlaWorld.__new__(CarlaWorld)
    world.carla = SimpleNamespace(Location=Location, Color=Color)
    world.world = SimpleNamespace(debug=Debug())
    world.no_rendering = False
    world._last_visualization_t = float("-inf")
    world.scripted_actors = []
    world.ego = FakeActor()
    frame = {"ego": {"v": 10.0}}
    state = {
        "route": [(10.0, 4.0), (20.0, 7.0)],
        "nav_state": 3,
        "remaining_m": 25.0,
        "behavior_state": 0,
        "target_speed_mps": 12.0,
        "aeb_state": 0,
        "safety_level": 0,
    }

    world.draw_adas_visualization(state, frame, {"stale": False}, 1.0)

    start, end = world.world.debug.lines[0]
    assert (start.x, start.y) == (10.0, -4.0)
    assert (end.x, end.y) == (20.0, -7.0)
    assert any(value == "NAV GOAL" for _, value in world.world.debug.strings)
    assert [value for _, value in world.world.debug.strings] == ["NAV GOAL"]

    # 5 Hz 限流：相邻同步 tick 不重复压入 CARLA debug RPC。
    world.draw_adas_visualization(state, frame, {"stale": False}, 1.1)
    assert len(world.world.debug.lines) == 1
