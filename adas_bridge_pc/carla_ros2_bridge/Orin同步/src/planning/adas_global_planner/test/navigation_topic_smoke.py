#!/usr/bin/env python3
"""ROS2 smoke test: lane graph + odometry + goal -> global route."""

import sys
import time

import rclpy
from adas_msgs.msg import (GlobalRoute, LaneConnection, LaneGraph, MapLane,
                           NavigationStatus, RoutePoint)
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


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
        self.goal_pub = self.create_publisher(
            PoseStamped, '/adas/navigation/goal_pose', 1)
        self.cancel_pub = self.create_publisher(
            Empty, '/adas/navigation/cancel', 1)
        self.path = None
        self.route = None
        self.status = None
        self.create_subscription(Path, '/adas/planning/global_route', self._path_cb, transient)
        self.create_subscription(
            GlobalRoute, '/adas/navigation/global_route', self._route_cb, transient)
        self.create_subscription(
            NavigationStatus, '/adas/navigation/status', self._status_cb, transient)

    def _path_cb(self, msg):
        self.path = msg

    def _route_cb(self, msg):
        self.route = msg

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
        unsafe_edge = LaneConnection()
        unsafe_edge.to_lane_id = 4
        unsafe_edge.maneuver = LaneConnection.STRAIGHT
        first.outgoing = [edge, unsafe_edge]
        second = MapLane()
        second.id = 2
        second.centerline = [pose(10, 0), pose(20, 0)]
        second.speed_limit_mps = 10.0
        unreachable = MapLane()
        unreachable.id = 3
        unreachable.centerline = [pose(0, 20), pose(10, 20)]
        unreachable.speed_limit_mps = 8.0
        unsafe = MapLane()
        unsafe.id = 4
        unsafe.centerline = [pose(100, 0), pose(110, 0)]
        unsafe.speed_limit_mps = 8.0
        graph.lanes = [first, second, unreachable, unsafe]
        self.map_pub.publish(graph)

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose = pose(1, 0)
        self.odom_pub.publish(odom)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose = pose(19, 0)
        self.goal_pub.publish(goal)

    def publish_unsafe_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose = pose(109, 0)
        self.goal_pub.publish(goal)


def main():
    rclpy.init()
    node = SmokeNode()
    deadline = time.monotonic() + 8.0
    published = False
    phase = 'route'
    valid_route_id = 0
    cancel_route_id = 0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not published and node.count_subscribers('/adas/navigation/goal_pose'):
                node.publish_inputs()
                published = True
            if (phase == 'route' and node.route and
                    node.route.status == GlobalRoute.STATUS_VALID and
                    len(node.route.points) >= 3 and
                    node.route.points[-1].maneuver == RoutePoint.MANEUVER_STOP and
                    node.path and len(node.path.poses) >= 3 and node.status and
                    node.status.state == NavigationStatus.DRIVING):
                valid_route_id = node.route.route_id
                node.cancel_pub.publish(Empty())
                phase = 'cancel'
            elif (phase == 'cancel' and node.route and
                  node.route.status == GlobalRoute.STATUS_CANCELLED and
                  node.route.route_id > valid_route_id and not node.route.points and
                  node.path is not None and not node.path.poses):
                cancel_route_id = node.route.route_id
                node.publish_unsafe_goal()
                phase = 'failure'
            elif (phase == 'failure' and node.route and
                  node.route.status == GlobalRoute.STATUS_FAILED and
                  node.route.route_id > cancel_route_id and not node.route.points and
                  node.path is not None and not node.path.poses and node.status and
                  node.status.state == NavigationStatus.FAILED):
                print('ROS2_LOOPBACK_PASS valid=%d cancelled=%d failed=%d' % (
                    valid_route_id, cancel_route_id, node.route.route_id))
                return 0
        print('ROS2_LOOPBACK_FAIL route=%r path=%r status=%r' % (
            node.route, node.path, node.status))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
