import os
import pathlib
import shutil
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from .health import (can_operstate_is_healthy, evaluate, gpu_temperature_c,
                     max_temperature_c, process_snapshot, read_meminfo,
                     swap_used_pct)


def memory_available_pct():
    values = read_meminfo()
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
        self.critical_processes = list(self.declare_parameter(
            "critical_processes", ["can_gateway_node", "safety_monitor_node",
                                    "command_gate_node", "trajectory_follower_node"]).value)
        self._previous_processes = {}
        self._previous_process_time = time.monotonic()
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
            "gpu_temperature_warn_c": 80.0,
            "gpu_temperature_error_c": 90.0,
            "swap_used_warn": 20.0,
            "swap_used_error": 50.0,
            "process_cpu_warn_pct": 80.0,
            "process_cpu_error_pct": 95.0,
            "process_rss_warn_mb": 1024.0,
            "process_rss_error_mb": 2048.0,
        }
        for name, default in tuple(self.thresholds.items()):
            self.thresholds[name] = float(self.declare_parameter(name, default).value)
        self.publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / self.rate_hz, self.on_timer)

    def collect(self):
        disk = shutil.disk_usage(self.disk_path)
        cpu_count = max(1, os.cpu_count() or 1)
        now = time.monotonic()
        elapsed = max(now - self._previous_process_time, 1e-6)
        processes = process_snapshot(self.critical_processes)
        for name, current in processes.items():
            previous = self._previous_processes.get(name)
            current["cpu_pct"] = (max(0.0, current["cpu_time_s"] -
                                       previous["cpu_time_s"]) / elapsed * 100.0
                                  if previous is not None else 0.0)
        self._previous_processes = processes
        self._previous_process_time = now
        can_state_path = pathlib.Path("/sys/class/net") / self.can_interface / "operstate"
        try:
            can_state = can_state_path.read_text(encoding="ascii").strip()
        except OSError:
            can_state = "missing"
        return {
            "memory_available_pct": memory_available_pct(),
            "disk_free_pct": 100.0 * disk.free / disk.total,
            "max_temperature_c": max_temperature_c(),
            "gpu_temperature_c": gpu_temperature_c(),
            "swap_used_pct": swap_used_pct(),
            "normalized_load_1m": os.getloadavg()[0] / cpu_count,
            "can_interface_up": can_operstate_is_healthy(can_state),
            "can_operstate": can_state,
            "log_path_writable": os.access(self.log_path, os.W_OK),
            "critical_processes": processes,
            "required_processes": self.critical_processes,
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
