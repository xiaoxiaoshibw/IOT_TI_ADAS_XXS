from types import SimpleNamespace

from adas_map.lane_graph import build_lane_graph, validate_lane_graph
from adas_map.waypoint_manager import encode_lane_key
import pytest


class Waypoint:
    def __init__(self, road, lane, s, x, next_waypoints=None, lane_type='Driving'):
        self.road_id = road
        self.section_id = 0
        self.lane_id = lane
        self.s = s
        self.lane_type = lane_type
        self.lane_change = 'None'
        self.is_junction = False
        self.transform = SimpleNamespace(
            location=SimpleNamespace(x=x, y=2.0, z=0.0),
            rotation=SimpleNamespace(yaw=0.0),
        )
        self._next = next_waypoints or []

    def next(self, _distance):  # noqa: A003 - mirrors the CARLA Waypoint API
        return self._next

    def get_left_lane(self):
        return None

    def get_right_lane(self):
        return None


def test_builds_valid_connected_graph_in_rep103_coordinates():
    second = [Waypoint(2, -1, 0.0, 4.0), Waypoint(2, -1, 2.0, 6.0)]
    first = [Waypoint(1, -1, 0.0, 0.0),
             Waypoint(1, -1, 2.0, 2.0, [second[0]])]
    graph = build_lane_graph(first + second, map_name='UnitTown')

    assert validate_lane_graph(graph) == {'lane_count': 2, 'connection_count': 1}
    assert graph['lanes'][0]['centerline'][0][1] == -2.0
    assert graph['lanes'][0]['outgoing'][0]['to_lane_id'] == encode_lane_key(2, 0, -1)
    assert len(graph['map_hash']) == 64


def test_filters_non_driving_and_short_lanes():
    driving = [Waypoint(1, -1, 0.0, 0.0), Waypoint(1, -1, 2.0, 2.0)]
    shoulder = [Waypoint(2, -1, 0.0, 0.0, lane_type='Shoulder'),
                Waypoint(2, -1, 2.0, 2.0, lane_type='Shoulder')]
    graph = build_lane_graph(driving + shoulder, map_name='UnitTown')
    assert len(graph['lanes']) == 1


@pytest.mark.parametrize('sample,speed', [(0.0, 10.0), (2.0, -1.0)])
def test_rejects_invalid_parameters(sample, speed):
    with pytest.raises(ValueError):
        build_lane_graph([], map_name='UnitTown', sample_distance_m=sample,
                         speed_limit_mps=speed)
