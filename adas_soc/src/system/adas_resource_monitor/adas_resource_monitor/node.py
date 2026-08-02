import os
import pathlib
import shutil

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from .health import evaluate, max_temperature_c


def memory_available_pct():
    values = {}
    with open("/proc/meminfo", encoding="ascii") as stream:
        for line in stream:
            name, value = line.split(":", 1)
            values[name] = int(value.strip().split()[0])
    return 100.0 * values["MemAvailable"] / values["MemTotal"]


class ResourceMonitorNode(Node):
    def __init__(self):
        super().__init__("resource_monitor")
        # Jetson HIL 主控制链路是 PEAK PCAN-USB → SocketCAN can1 @ 500k，
        # 由 can_hil.yaml 的 /** can_interface: can1 覆盖；此默认值仅在无 YAML 的裸跑场景生效。
        self.can_interface = self.declare_parameter("can_interface", "can0").value
        self.disk_path = self.declare_parameter("disk_path", "/var/log/adas").value
        self.log_path = self.declare_parameter("log_path", "/var/log/adas").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 1.0).value)
        if self.rate_hz <= 0.0 or self.rate_hz > 10.0:
            raise ValueError("rate_hz must be in (0, 10]")
        self.thresholds = {
            "memory_available_warn": 15.0,
            "memory_available_error": 7.0,
            "disk_free_warn": 15.0,
            "disk_free_error": 7.0,
            "temperature_warn_c": 80.0,
            "temperature_error_c": 90.0,
            "normalized_load_warn": 1.0,
            "normalized_load_error": 1.5,
        }
        for name, default in tuple(self.thresholds.items()):
            self.thresholds[name] = float(self.declare_parameter(name, default).value)
        self.publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / self.rate_hz, self.on_timer)

    def collect(self):
        disk = shutil.disk_usage(self.disk_path)
        cpu_count = max(1, os.cpu_count() or 1)
        can_state_path = pathlib.Path("/sys/class/net") / self.can_interface / "operstate"
        try:
            can_state = can_state_path.read_text(encoding="ascii").strip()
        except OSError:
            can_state = "missing"
        return {
            "memory_available_pct": memory_available_pct(),
            "disk_free_pct": 100.0 * disk.free / disk.total,
            "max_temperature_c": max_temperature_c(),
            "normalized_load_1m": os.getloadavg()[0] / cpu_count,
            "can_interface_up": can_state in ("up", "unknown"),
            "can_operstate": can_state,
            "log_path_writable": os.access(self.log_path, os.W_OK),
        }

    def on_timer(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "resource_monitor: platform_health"
        status.hardware_id = "jetson-orin-nano"
        try:
            metrics = self.collect()
            level, status.message = evaluate(metrics, self.thresholds)
            # Humble's Python binding represents diagnostic_msgs/byte as a
            # one-byte object, while newer generated bindings accept int.
            try:
                status.level = level
            except (AssertionError, TypeError):
                status.level = bytes([level])
            status.values = [KeyValue(key=name, value=str(value))
                             for name, value in metrics.items()]
        except (OSError, KeyError, ValueError) as error:
            status.level = DiagnosticStatus.ERROR
            status.message = f"resource collection failed: {error}"
        message.status = [status]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
