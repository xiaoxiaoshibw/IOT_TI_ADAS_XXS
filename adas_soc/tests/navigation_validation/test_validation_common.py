from types import SimpleNamespace

import pytest

from validation_common import (
    encoded_road_id,
    navigation_metrics,
    nearest_route_point,
    validate_route,
)


def test_encoded_road_id_preserves_signed_value():
    assert encoded_road_id((123 << 24) | 7) == 123
    assert encoded_road_id(-7) == 0


def test_signed_lateral_error_is_rep103_left_positive():
    points = [SimpleNamespace(x=0.0, y=0.0, yaw=0.0)]
    _, _, error = nearest_route_point(points, 2.0, 1.5)
    assert error == pytest.approx(1.5)


def test_metrics_do_not_substitute_missing_measurements():
    result = navigation_metrics(
        [
            {"lateral_error": "1", "speed_error": "2", "vehicle_x": "0", "vehicle_y": "0"},
            {"lateral_error": "-1", "speed_error": "", "vehicle_x": "3", "vehicle_y": "4"},
        ]
    )
    assert result["lateral_error_rms_m"] == pytest.approx(1.0)
    assert result["speed_error_rms_mps"] == pytest.approx(2.0)
    assert result["travel_distance_m"] == pytest.approx(5.0)


def _route(points):
    return SimpleNamespace(
        points=points,
        status=2,
        route_id=1,
        frame_id="map",
        header=SimpleNamespace(frame_id="map"),
        length=sum(
            ((b.x - a.x) ** 2 + (b.y - a.y) ** 2) ** 0.5
            for a, b in zip(points, points[1:])
        ),
    )


def _point(x, y, yaw=0.0, maneuver=1):
    lane_id = (42 << 24) | 1
    return SimpleNamespace(
        x=x,
        y=y,
        yaw=yaw,
        speed_limit=13.9,
        maneuver=maneuver,
        lane_id=lane_id,
        road_id=42,
    )


def test_route_safety_gate_reports_phase31_style_gap():
    route = _route([
        _point(384.3917, 57.7676),
        _point(388.0449, 77.7402, maneuver=6),
    ])
    check = validate_route(route)
    assert not check.ok
    assert check.maximum_adjacent_gap_m == pytest.approx(20.3039558)
    assert check.largest_gap_point_index == 1
    assert "adjacent gap" in " ".join(check.errors)


def test_route_safety_gate_accepts_forward_two_meter_sampling():
    check = validate_route(_route([
        _point(0.0, 0.0),
        _point(2.0, 0.0),
        _point(4.0, 0.0, maneuver=6),
    ]))
    assert check.ok
    assert check.maximum_adjacent_gap_m == pytest.approx(2.0)
    assert check.maximum_reverse_progress_m == 0.0
