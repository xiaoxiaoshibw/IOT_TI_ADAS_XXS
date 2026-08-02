#!/usr/bin/env python3
"""Publish a bounded synthetic XYZI cloud for LiDAR ingress smoke tests."""

import argparse
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(node, point_count):
    message = PointCloud2()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'lidar_front'
    message.height = 1
    message.width = point_count
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12,
                   datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = message.point_step * point_count
    point = struct.pack('<ffff', 10.0, 0.0, 0.0, 0.5)
    message.data = point * point_count
    message.is_dense = True
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--points', type=int, default=1000)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.rate <= 0.0 or args.points <= 0:
        raise SystemExit('duration, rate, and points must be positive')

    rclpy.init()
    node = Node('test_pointcloud_publisher')
    publisher = node.create_publisher(
        PointCloud2, '/adas/sensors/front/points', qos_profile_sensor_data)
    deadline = time.monotonic() + args.duration
    period = 1.0 / args.rate
    sent = 0
    try:
        while time.monotonic() < deadline:
            publisher.publish(make_cloud(node, args.points))
            sent += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'published {sent} synthetic point clouds')


if __name__ == '__main__':
    main()
