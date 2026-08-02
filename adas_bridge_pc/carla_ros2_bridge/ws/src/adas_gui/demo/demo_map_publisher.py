#!/usr/bin/env python3
"""Synthetic lane-graph + odometry publisher for exercising adas_gui without CARLA.

Publishes a rectangular lane loop (transient_local, same topic/QoS as the CARLA
bridge) and a 10 Hz static odometry so the global planner can snap a start
pose. Click anywhere within snap distance of a lane in the GUI to plan a route.
"""

import math

import rclpy
from adas_msgs.msg import LaneConnection, LaneGraph, MapLane
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def pose(x, y):
    msg = Pose()
    msg.position.x = float(x)
    msg.position.y = float(y)
    msg.orientation.w = 1.0
    return msg


def line(x0, y0, x1, y1, step=5.0):
    length = math.hypot(x1 - x0, y1 - y0)
    count = max(2, int(length / step) + 1)
    return [pose(x0 + (x1 - x0) * i / (count - 1),
                 y0 + (y1 - y0) * i / (count - 1)) for i in range(count)]


def lane(lane_id, centerline, to_ids, junction=False):
    msg = MapLane()
    msg.id = lane_id
    msg.centerline = centerline
    msg.speed_limit_mps = 8.0
    msg.junction = junction
    for to_id in to_ids:
        edge = LaneConnection()
        edge.to_lane_id = to_id
        edge.maneuver = LaneConnection.STRAIGHT
        msg.outgoing.append(edge)
    return msg


class DemoMapPublisher(Node):
    def __init__(self):
        super().__init__("demo_map_publisher")
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        graph = LaneGraph()
        graph.header.frame_id = "map"
        graph.map_id = "demo_loop"
        graph.map_hash = "demo"
        # 双向矩形环 + 一条中间横穿道，供双向路径规划
        graph.lanes = [
            lane(1, line(0, 0, 120, 0), [2]),
            lane(2, line(120, 0, 120, 80), [3], junction=True),
            lane(3, line(120, 80, 0, 80), [4]),
            lane(4, line(0, 80, 0, 0), [1], junction=True),
            lane(5, line(120, 40, 0, 40), [4]),        # 横穿：东→西
            lane(6, line(120, 0, 120, 40), [5], junction=True),
            lane(7, line(0, 40, 0, 0), [1]),
        ]
        # 让 2 号车道也能进入横穿道起点、5 号能接 7 号
        graph.lanes[1].outgoing.append(LaneConnection(to_lane_id=6,
                                                      maneuver=LaneConnection.STRAIGHT))
        graph.lanes[4].outgoing.append(LaneConnection(to_lane_id=7,
                                                      maneuver=LaneConnection.STRAIGHT))
        self.map_pub = self.create_publisher(LaneGraph, "/adas/map/lane_graph", transient)
        self.map_pub.publish(graph)
        self.get_logger().info("demo lane graph published (7 lanes)")

        self.odom_pub = self.create_publisher(
            Odometry, "/adas/localization/kinematic_state", 10)
        # 沿外环 (0,0)->(120,0)->(120,80)->(0,80)->(0,0) 匀速行驶
        self.waypoints = [(0.0, 0.0), (120.0, 0.0), (120.0, 80.0), (0.0, 80.0)]
        self.distance = 0.0
        self.speed_mps = 5.0
        self.create_timer(0.1, self.publish_odom)

    def loop_pose(self, distance):
        segments = []
        total = 0.0
        for i, start in enumerate(self.waypoints):
            end = self.waypoints[(i + 1) % len(self.waypoints)]
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            segments.append((start, end, length))
            total += length
        distance %= total
        for start, end, length in segments:
            if distance <= length:
                ratio = distance / length
                x = start[0] + (end[0] - start[0]) * ratio
                y = start[1] + (end[1] - start[1]) * ratio
                yaw = math.atan2(end[1] - start[1], end[0] - start[0])
                return x, y, yaw
            distance -= length
        return self.waypoints[0][0], self.waypoints[0][1], 0.0

    def publish_odom(self):
        self.distance += self.speed_mps * 0.1
        x, y, yaw = self.loop_pose(self.distance)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.pose.pose = pose(x, y)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = self.speed_mps
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = DemoMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
