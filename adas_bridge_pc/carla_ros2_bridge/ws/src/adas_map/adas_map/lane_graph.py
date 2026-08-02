"""Build the runtime ADAS lane graph from CARLA waypoints."""

import hashlib
import math

from .opendrive_parser import parse_opendrive
from .waypoint_manager import (
    encode_lane_key,
    group_driving_waypoints,
    lane_key,
    right_handed_yaw,
    to_right_handed,
)

DEFAULT_SAMPLE_DISTANCE_M = 2.0
DEFAULT_SPEED_LIMIT_MPS = 13.9

MANEUVER_STRAIGHT = 0
MANEUVER_LEFT = 1
MANEUVER_RIGHT = 2
MANEUVER_LANE_CHANGE_LEFT = 3
MANEUVER_LANE_CHANGE_RIGHT = 4
TURN_YAW_THRESHOLD_RAD = math.radians(30.0)


def _norm_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _same_direction(first_lane_id, second_lane_id):
    return (first_lane_id > 0) == (second_lane_id > 0)


def _forward_maneuver(last_waypoint, destination):
    if not bool(getattr(destination[-1], 'is_junction', False)):
        return MANEUVER_STRAIGHT
    delta = _norm_angle(
        right_handed_yaw(destination[-1]) - right_handed_yaw(last_waypoint))
    if delta > TURN_YAW_THRESHOLD_RAD:
        return MANEUVER_LEFT
    if delta < -TURN_YAW_THRESHOLD_RAD:
        return MANEUVER_RIGHT
    return MANEUVER_STRAIGHT


def _forward_connections(last_waypoint, sample_distance_m, groups):
    connections = []
    seen = {lane_key(last_waypoint)}
    try:
        successors = last_waypoint.next(max(sample_distance_m, 0.5))
    except RuntimeError:
        successors = []
    for successor in successors:
        key = lane_key(successor)
        if key in seen or key not in groups:
            continue
        seen.add(key)
        connections.append((key, _forward_maneuver(last_waypoint, groups[key])))
    return connections


def _lane_change_connections(last_waypoint, groups):
    connections = []
    permission = str(getattr(last_waypoint, 'lane_change', None))
    for allowed, accessor, maneuver in (
            (permission in ('Left', 'Both'), 'get_left_lane',
             MANEUVER_LANE_CHANGE_LEFT),
            (permission in ('Right', 'Both'), 'get_right_lane',
             MANEUVER_LANE_CHANGE_RIGHT)):
        if not allowed:
            continue
        try:
            adjacent = getattr(last_waypoint, accessor)()
        except RuntimeError:
            adjacent = None
        if (adjacent is not None
                and str(getattr(adjacent, 'lane_type', '')) == 'Driving'
                and _same_direction(adjacent.lane_id, last_waypoint.lane_id)
                and lane_key(adjacent) in groups):
            connections.append((lane_key(adjacent), maneuver))
    return connections


def _topology_hash(map_name, lanes, sample_distance_m):
    """Create a deterministic fallback identity when raw OpenDRIVE is unavailable."""
    digest = hashlib.sha256()
    digest.update(('%s|%.6f' % (map_name, sample_distance_m)).encode('utf-8'))
    for lane in sorted(lanes, key=lambda item: item['id']):
        digest.update(('|%d|%.3f|%d' % (
            lane['id'], lane['speed_limit_mps'], lane['junction'])).encode('ascii'))
        for x, y, yaw in lane['centerline']:
            digest.update(('|%.4f,%.4f,%.6f' % (x, y, yaw)).encode('ascii'))
        for edge in sorted(lane['outgoing'], key=lambda item: (
                item['to_lane_id'], item['maneuver'])):
            digest.update(('|>%d,%d,%.3f' % (
                edge['to_lane_id'], edge['maneuver'],
                edge['extra_cost_m'])).encode('ascii'))
    return digest.hexdigest()


def validate_lane_graph(graph):
    """Return topology statistics or raise ValueError for an unsafe graph."""
    lane_ids = [lane['id'] for lane in graph.get('lanes', [])]
    if not graph.get('map_id'):
        raise ValueError('map_id is empty')
    if not graph.get('map_hash'):
        raise ValueError('map_hash is empty')
    if not lane_ids:
        raise ValueError('lane graph contains no driving lanes')
    if len(set(lane_ids)) != len(lane_ids):
        raise ValueError('lane graph contains duplicate lane ids')
    known = set(lane_ids)
    connection_count = 0
    for lane in graph['lanes']:
        if len(lane['centerline']) < 2:
            raise ValueError('lane %d has fewer than two points' % lane['id'])
        if lane['speed_limit_mps'] <= 0.0:
            raise ValueError('lane %d has invalid speed limit' % lane['id'])
        seen = set()
        for edge in lane['outgoing']:
            identity = (edge['to_lane_id'], edge['maneuver'])
            if edge['to_lane_id'] not in known:
                raise ValueError('lane %d has a dangling connection' % lane['id'])
            if edge['to_lane_id'] == lane['id']:
                raise ValueError('lane %d has a self connection' % lane['id'])
            if identity in seen:
                raise ValueError('lane %d has a duplicate connection' % lane['id'])
            seen.add(identity)
            connection_count += 1
    return {'lane_count': len(lane_ids), 'connection_count': connection_count}


def build_lane_graph(waypoints, map_name='', sample_distance_m=DEFAULT_SAMPLE_DISTANCE_M,
                     speed_limit_mps=DEFAULT_SPEED_LIMIT_MPS, map_hash=None):
    """Convert the CARLA Waypoint API subset into the stable lane graph dictionary."""
    if not math.isfinite(sample_distance_m) or sample_distance_m <= 0.0:
        raise ValueError('sample_distance_m must be positive and finite')
    if not math.isfinite(speed_limit_mps) or speed_limit_mps <= 0.0:
        raise ValueError('speed_limit_mps must be positive and finite')

    groups = group_driving_waypoints(waypoints)
    key_to_id = {key: encode_lane_key(*key) for key in groups}
    lanes = []
    for key in sorted(groups):
        lane_waypoints = groups[key]
        last_waypoint = lane_waypoints[-1]
        outgoing = []
        connections = (_forward_connections(last_waypoint, sample_distance_m, groups)
                       + _lane_change_connections(last_waypoint, groups))
        seen = set()
        for destination, maneuver in connections:
            identity = (key_to_id[destination], maneuver)
            if identity in seen:
                continue
            seen.add(identity)
            outgoing.append({
                'to_lane_id': identity[0],
                'maneuver': maneuver,
                'extra_cost_m': 0.0,
            })
        lanes.append({
            'id': key_to_id[key],
            'centerline': [to_right_handed(wp.transform) for wp in lane_waypoints],
            'speed_limit_mps': speed_limit_mps,
            'junction': any(bool(getattr(wp, 'is_junction', False))
                            for wp in lane_waypoints),
            'outgoing': outgoing,
        })

    resolved_hash = map_hash or _topology_hash(map_name, lanes, sample_distance_m)
    graph = {'map_id': map_name, 'map_hash': resolved_hash, 'lanes': lanes}
    validate_lane_graph(graph)
    return graph


def export_lane_graph(carla_map, sample_distance_m=DEFAULT_SAMPLE_DISTANCE_M,
                      speed_limit_mps=DEFAULT_SPEED_LIMIT_MPS):
    """Build and validate a graph from a real ``carla.Map`` instance."""
    raw_opendrive = carla_map.to_opendrive()
    metadata = parse_opendrive(raw_opendrive, fallback_name=carla_map.name)
    map_name = carla_map.name or metadata.map_name
    waypoints = carla_map.generate_waypoints(sample_distance_m)
    return build_lane_graph(
        waypoints,
        map_name=map_name,
        sample_distance_m=sample_distance_m,
        speed_limit_mps=speed_limit_mps,
        map_hash=metadata.content_hash,
    )
