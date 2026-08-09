import math
from types import SimpleNamespace

from adas_carla_bridge.carla_world import (
    _lane_reference_valid,
    _select_forward_waypoint,
)


def waypoint(x, y, yaw_deg):
    return SimpleNamespace(transform=SimpleNamespace(
        location=SimpleNamespace(x=float(x), y=float(y)),
        rotation=SimpleNamespace(yaw=float(yaw_deg))))


def test_junction_successor_rejects_perpendicular_crossing_lane():
    ego = SimpleNamespace(x=4.0, y=0.1)
    straight = waypoint(5.0, 0.0, 2.0)
    crossing = waypoint(4.1, 0.0, 90.0)

    selected = _select_forward_waypoint(
        [crossing, straight], ego, math.radians(1.0), 0.0)

    assert selected is straight


def test_junction_successor_returns_none_when_only_crossing_lane_exists():
    ego = SimpleNamespace(x=4.0, y=0.0)
    crossing = waypoint(4.1, 0.0, -90.0)

    assert _select_forward_waypoint([crossing], ego, 0.0, 0.0) is None


def test_lane_reference_invalid_only_for_large_combined_disagreement():
    assert _lane_reference_valid(0.2, math.radians(80.0))
    assert _lane_reference_valid(2.0, math.radians(20.0))
    assert not _lane_reference_valid(2.0, math.radians(80.0))
