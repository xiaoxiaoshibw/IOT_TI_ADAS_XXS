"""CARLA waypoint grouping and coordinate conversion helpers."""

import math


def lane_key(waypoint):
    """Return the OpenDRIVE road/section/lane identity of a waypoint."""
    return (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))


def encode_lane_key(road_id, section_id, lane_id):
    """Encode an OpenDRIVE lane identity into the stable ROS int64 lane id."""
    return (int(road_id) << 24) | ((int(section_id) & 0xFF) << 16) | (
        (int(lane_id) + 32768) & 0xFFFF)


def group_driving_waypoints(waypoints, minimum_points=2):
    """Group waypoints by physical lane and order every group by longitudinal s."""
    groups = {}
    for waypoint in waypoints:
        if str(getattr(waypoint, 'lane_type', 'Driving')) != 'Driving':
            continue
        groups.setdefault(lane_key(waypoint), []).append(waypoint)
    for key in groups:
        groups[key].sort(key=lambda waypoint: waypoint.s)
    return {key: value for key, value in groups.items()
            if len(value) >= minimum_points}


def to_right_handed(transform):
    """Convert CARLA left-handed transform to REP-103 x/y/yaw."""
    yaw = -math.radians(float(transform.rotation.yaw))
    return (float(transform.location.x), -float(transform.location.y), yaw)


def right_handed_yaw(waypoint):
    """Return waypoint heading in REP-103 radians."""
    return -math.radians(float(waypoint.transform.rotation.yaw))
