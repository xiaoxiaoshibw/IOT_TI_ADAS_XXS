#!/usr/bin/env python3
"""ROS2 smoke test: lane graph + odometry + goal -> global route."""

import sys
import time

import rclpy
from adas_msgs.msg import LaneConnection, LaneGraph, MapLane, NavigationStatus
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def pose(x, y):
    msg = Pose()
    msg.position.x = float(x)
    msg.position.y = float(y)
    msg.orientation.w = 1.0
    return msg


class SmokeNode(Node):
    def __init__(self):
        super().__init__('global_planner_smoke')
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(LaneGraph, '/adas/map/lane_graph', transient)
        self.odom_pub = self.create_publisher(
            Odometry, '/adas/localization/kinematic_state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/adas/navigation/goal', 1)
        self.path = None
        self.status = None
        self.create_subscription(Path, '/adas/planning/global_route', self._path_cb, transient)
        self.create_subscription(
            NavigationStatus, '/adas/navigation/status', self._status_cb, transient)

    def _path_cb(self, msg):
        self.path = msg

    def _status_cb(self, msg):
        self.status = msg

    def publish_inputs(self):
        graph = LaneGraph()
        graph.header.frame_id = 'map'
        graph.map_id = 'smoke_map'
        graph.map_hash = 'abc123'
        first = MapLane()
        first.id = 1
        first.centerline = [pose(0, 0), pose(10, 0)]
        first.speed_limit_mps = 10.0
        edge = LaneConnection()
        edge.to_lane_id = 2
        edge.maneuver = LaneConnection.STRAIGHT
        first.outgoing = [edge]
        second = MapLane()
        second.id = 2
        second.centerline = [pose(10, 0), pose(20, 0)]
        second.speed_limit_mps = 10.0
        graph.lanes = [first, second]
        self.map_pub.publish(graph)

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose = pose(1, 0)
        self.odom_pub.publish(odom)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose = pose(19, 0)
        self.goal_pub.publish(goal)


def main():
    rclpy.init()
    node = SmokeNode()
    deadline = time.monotonic() + 8.0
    published = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not published and node.count_subscribers('/adas/navigation/goal'):
                node.publish_inputs()
                published = True
            if (node.path and len(node.path.poses) >= 3 and node.status and
                    node.status.state == NavigationStatus.DRIVING):
                print('ROS2_LOOPBACK_PASS poses=%d route_id=%s' % (
                    len(node.path.poses), node.status.route_id))
                return 0
        print('ROS2_LOOPBACK_FAIL path=%r status=%r' % (node.path, node.status))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
