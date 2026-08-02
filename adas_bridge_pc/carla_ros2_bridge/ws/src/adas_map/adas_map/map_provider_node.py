#!/usr/bin/env python3
"""ROS 2 provider for the current CARLA OpenDRIVE lane graph."""

import importlib
import math

from adas_msgs.msg import LaneConnection, LaneGraph, MapLane
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .lane_graph import export_lane_graph, validate_lane_graph

TOPIC_LANE_GRAPH = '/adas/map/lane_graph'


def graph_to_message(graph, stamp):
    """Convert the pure-Python graph representation to the stable ROS contract."""
    validate_lane_graph(graph)
    message = LaneGraph()
    message.header.stamp = stamp
    message.header.frame_id = 'map'
    message.map_id = graph['map_id']
    message.map_hash = graph['map_hash']
    for lane_data in graph['lanes']:
        lane = MapLane()
        lane.id = int(lane_data['id'])
        lane.speed_limit_mps = float(lane_data['speed_limit_mps'])
        lane.junction = bool(lane_data['junction'])
        for x, y, yaw in lane_data['centerline']:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            lane.centerline.append(pose)
        for edge_data in lane_data['outgoing']:
            edge = LaneConnection()
            edge.to_lane_id = int(edge_data['to_lane_id'])
            edge.maneuver = int(edge_data['maneuver'])
            edge.extra_cost_m = float(edge_data['extra_cost_m'])
            lane.outgoing.append(edge)
        message.lanes.append(lane)
    return message


class MapProviderNode(Node):
    """Connect to CARLA, validate OpenDRIVE, and publish one latched lane graph."""

    def __init__(self):
        super().__init__('adas_map_provider')
        self.declare_parameter('carla.host', '127.0.0.1')
        self.declare_parameter('carla.port', 2000)
        self.declare_parameter('carla.timeout_s', 5.0)
        self.declare_parameter('sample_distance_m', 2.0)
        self.declare_parameter('default_speed_limit_mps', 13.9)
        self.declare_parameter('retry_period_s', 2.0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(LaneGraph, TOPIC_LANE_GRAPH, qos)
        self._published_hash = ''
        retry_period = float(self.get_parameter('retry_period_s').value)
        if retry_period <= 0.0:
            raise ValueError('retry_period_s must be positive')
        self._timer = self.create_timer(retry_period, self._load_and_publish)
        self._load_and_publish()

    def _load_and_publish(self):
        try:
            carla = importlib.import_module('carla')
            client = carla.Client(
                str(self.get_parameter('carla.host').value),
                int(self.get_parameter('carla.port').value),
            )
            client.set_timeout(float(self.get_parameter('carla.timeout_s').value))
            graph = export_lane_graph(
                client.get_world().get_map(),
                sample_distance_m=float(self.get_parameter('sample_distance_m').value),
                speed_limit_mps=float(
                    self.get_parameter('default_speed_limit_mps').value),
            )
        except (ImportError, OSError, RuntimeError, ValueError) as error:
            self.get_logger().warn('地图尚未就绪：%s' % error)
            return

        if graph['map_hash'] == self._published_hash:
            return
        message = graph_to_message(graph, self.get_clock().now().to_msg())
        self._publisher.publish(message)
        self._published_hash = graph['map_hash']
        connection_count = sum(len(lane.outgoing) for lane in message.lanes)
        self.get_logger().info(
            '地图就绪 %s：%d 车道 / %d 连接 / sha256=%s…'
            % (message.map_id, len(message.lanes), connection_count,
               message.map_hash[:12]))


def main(args=None):
    rclpy.init(args=args)
    node = MapProviderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
