from __future__ import annotations

from adas_msgs.msg import ActuationCommand, BehaviorState, GlobalRoute, Trajectory

from no_route_control_audit import event_payload


def test_invalid_global_route_is_explicitly_not_valid() -> None:
    message = GlobalRoute()
    message.route_id = 0
    message.status = GlobalRoute.STATUS_INVALID
    row = event_payload("/adas/navigation/global_route", message)
    assert row["route_id"] == 0
    assert row["point_count"] == 0
    assert row["valid"] == 0


def test_behavior_target_speed_is_recorded() -> None:
    message = BehaviorState()
    message.target_speed_mps = 15.0
    row = event_payload("/adas/planning/behavior", message)
    assert row["target_speed_mps"] == "15.000000"


def test_empty_trajectory_is_invalid_zero_speed() -> None:
    row = event_payload("/adas/planning/trajectory", Trajectory())
    assert row["point_count"] == 0
    assert row["target_speed_mps"] == "0.000000"
    assert row["valid"] == 0


def test_actuation_values_are_recorded_without_conversion() -> None:
    message = ActuationCommand()
    message.throttle = 0.17
    message.brake = 0.0
    message.steer = -0.02
    row = event_payload("/adas/mcu/actuation_feedback", message)
    assert row["throttle"] == "0.170000"
    assert row["brake"] == "0.000000"
    assert row["steering"] == "-0.0200000"
