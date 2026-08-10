#!/usr/bin/env python3
"""
ADAS MQTT Bridge — ROS2 Lifecycle Node
=======================================
在 Jetson Orin Nano 上作为 ROS2 生命周期节点运行。
遵循 SoC 栈的安全链激活顺序：在 vehicle_interface 之后激活。

订阅 /adas/* 话题 → 桥接到 MQTT 云平台。
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import json
import os

from adas_mqtt_bridge.mqtt_bridge import AdasMqttClient


class MqttBridgeLifecycleNode(LifecycleNode):
    """MQTT 桥接器 — 生命周期节点"""

    def __init__(self):
        super().__init__('adas_mqtt_bridge')
        self._mqtt = None
        self._subscribers = []
        self._seq = 0

        # 声明参数
        self.declare_parameter('mqtt_broker', 'broker.emqx.io')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('topic_prefix', 'adas/v1')
        self.declare_parameter('client_id', 'adas_orin_soc')
        self.declare_parameter('publish_rate', 10.0)

        self.get_logger().info('MQTT Bridge lifecycle node created')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """配置阶段：读取参数，创建 MQTT 客户端"""
        self.get_logger().info('Configuring MQTT Bridge...')

        broker = self.get_parameter('mqtt_broker').value
        port = self.get_parameter('mqtt_port').value
        prefix = self.get_parameter('topic_prefix').value
        client_id = self.get_parameter('client_id').value

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

        self._rate_sub = self.create_subscription(
            String, '/adas/vehicle/actuation_cmd',
            self._on_actuation, 10)
        self._subscribers.append(self._rate_sub)

        self.get_logger().info(
            f'Configured: {broker}:{port}, prefix={prefix}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """激活阶段：连接 MQTT Broker 并启动心跳"""
        self.get_logger().info('Activating MQTT Bridge...')
        self._mqtt.connect()

        self._hb_timer = self.create_timer(5.0, self._publish_heartbeat)

        # 初始心跳
        self._publish_heartbeat()
        self.get_logger().info('MQTT Bridge activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """停用阶段：断开 MQTT"""
        self.get_logger().info('Deactivating MQTT Bridge...')
        if self._hb_timer:
            self._hb_timer.cancel()
        if self._mqtt:
            self._mqtt.disconnect()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """清理阶段"""
        self._subscribers.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """关闭"""
        self._subscribers.clear()
        if self._mqtt:
            self._mqtt.disconnect()
        return TransitionCallbackReturn.SUCCESS

    def _on_actuation(self, msg):
        """执行指令 → MQTT"""
        self._mqtt.publish('vehicle/actuation', {
            'source': 'ros2',
            'seq': self._next_seq(),
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'data': str(msg.data)[:200],
        })

    def _publish_heartbeat(self):
        """ROS2 节点心跳"""
        uptime = self.get_clock().now().nanoseconds / 1e9
        self._mqtt.publish('system/status', {
            'type': 'heartbeat',
            'node': 'adas_mqtt_bridge',
            'status': 'active',
            'lifecycle_state': self._state_machine.current_state[1],
            'uptime_s': round(uptime, 1),
            'seq': self._next_seq(),
        })

    def _next_seq(self):
        self._seq = (self._seq + 1) % 65536
        return self._seq


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeLifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
