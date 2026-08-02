#!/usr/bin/env python3
"""
ADAS MQTT Bridge — ROS2 Node Wrapper (Simplified)
==================================================
在 Jetson Orin Nano 上作为 ROS2 节点运行。
订阅 ADAS 话题，桥接到 MQTT 云平台。

正式部署：
  1. 复制到 SoC 工作区：cp mqtt_bridge.py ~/adas/adas_soc_ws/src/system/adas_mqtt_bridge/
  2. 直接 ros2 run 或做成 systemd 服务

独立调试：
  ./mqtt_bridge.py --standalone
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32
import json
import os
import sys

# 确保能找到 mqtt_bridge 模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mqtt_bridge import AdasMqttClient


class MqttBridgeNode(Node):
    """ROS2 节点 — 订阅 ADAS 话题 → MQTT"""

    def __init__(self):
        super().__init__('adas_mqtt_bridge')

        # 声明参数
        self.declare_parameter('mqtt_broker', 'broker.emqx.io')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('topic_prefix', 'adas/v1')
        self.declare_parameter('client_id', 'adas_orin_soc')

        broker = self.get_parameter('mqtt_broker').value
        port = self.get_parameter('mqtt_port').value
        prefix = self.get_parameter('topic_prefix').value
        client_id = self.get_parameter('client_id').value

        self.get_logger().info(
            f'MQTT Bridge starting — {broker}:{port}, prefix={prefix}'
        )

        # 创建 MQTT 客户端
        config = {
            'mqtt': {
                'broker': broker,
                'port': port,
                'topic_prefix': prefix,
                'client_id_prefix': client_id,
                'qos': 1,
            }
        }
        self._mqtt = AdasMqttClient(config)
        self._mqtt.connect()
        self._seq = 0

        # 订阅话题
        self._setup_subscribers()

        # 心跳定时器
        self._hb_timer = self.create_timer(5.0, self._publish_heartbeat)

        self.get_logger().info('✓ MQTT Bridge ROS2 node ready')

    def _setup_subscribers(self):
        """订阅 ADAS 核心话题"""
        # 执行指令话题
        self.create_subscription(
            String,
            '/adas/vehicle/actuation_cmd',
            self._on_actuation,
            10
        )
        # 里程计话题
        self.create_subscription(
            String,
            '/adas/localization/kinematic_state',
            self._on_odometry,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        )

    def _on_actuation(self, msg):
        """执行指令 → MQTT"""
        self._mqtt.publish('vehicle/actuation', {
            'source': 'ros2',
            'seq': self._next_seq(),
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'data': str(msg.data)[:200],
        })

    def _on_odometry(self, msg):
        """里程计 → MQTT（提取速度信息）"""
        try:
            data = json.loads(msg.data) if isinstance(msg.data, str) else {}
            self._mqtt.publish('vehicle/state', {
                'speed': data.get('speed', 0),
                'seq': self._next_seq(),
            })
        except Exception:
            pass

    def _publish_heartbeat(self):
        """系统心跳"""
        self._mqtt.publish('system/status', {
            'type': 'heartbeat',
            'node': 'adas_mqtt_bridge',
            'status': 'active',
            'uptime_s': self.get_clock().now().nanoseconds / 1e9,
            'seq': self._next_seq(),
        })

    def _next_seq(self):
        self._seq = (self._seq + 1) % 65536
        return self._seq

    def destroy_node(self):
        self._mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
