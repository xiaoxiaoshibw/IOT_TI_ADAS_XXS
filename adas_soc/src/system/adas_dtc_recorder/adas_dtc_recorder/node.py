import json
import pathlib
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from adas_msgs.msg import ActuationCommand, GateStatus, McuStatus, SafetyStatus

from .store import DtcStore


class DtcRecorderNode(Node):
    def __init__(self):
        super().__init__("dtc_recorder")
        default_catalog = pathlib.Path(
            get_package_share_directory("adas_dtc_recorder")) / "fault_catalog.json"
        history_path = self.declare_parameter(
            "history_path", "/var/log/adas/dtc_history.json").value
        catalog_path = pathlib.Path(self.declare_parameter(
            "catalog_path", str(default_catalog)).value)
        max_records = int(self.declare_parameter("max_records", 128).value)
        if max_records < 1 or max_records > 1024:
            raise ValueError("max_records must be in [1, 1024]")
        catalog_payload = json.loads(catalog_path.read_text(encoding="utf-8"))
        if catalog_payload.get("schema_version") != 1:
            raise ValueError("unsupported fault catalog schema")
        self.catalog = catalog_payload["faults"]
        self.store = DtcStore(history_path, self.catalog, max_records)
        self.by_source = {item["source"]: item for item in self.catalog}
        self.latest = {
            "speed_mps": None, "steer": None, "throttle": None, "brake": None,
            "control_source": None, "safety_level": None,
            "mcu_state": None, "mcu_active_source": None, "mcu_degrade_reason": None,
        }
        self.seen_in_cycle = set()
        self.mcu_faults = [(item["mcu_fault_bit"], item["code"])
                           for item in self.catalog if "mcu_fault_bit" in item]

        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 50)
        self.create_subscription(Odometry, "/adas/localization/kinematic_state",
                                 self.on_odometry, 10)
        self.create_subscription(ActuationCommand, "/adas/mcu/actuation_feedback",
                                 self.on_actuation, 10)
        self.create_subscription(GateStatus, "/adas/control/gate/status", self.on_gate, 10)
        self.create_subscription(SafetyStatus, "/adas/system/safety_status",
                                 self.on_safety, 10)
        self.create_subscription(McuStatus, "/adas/mcu/status", self.on_mcu_status, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, "/adas/diagnostics/dtc_history", qos)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", QoSProfile(depth=10))
        self.create_timer(1.0, self.flush_and_publish)

    def freeze_frame(self, status):
        values = {item.key: item.value for item in status.values[:32]}
        return dict(self.latest, diagnostic_name=status.name,
                    diagnostic_level=int(status.level), message=status.message,
                    values=values)

    def source_from_name(self, name):
        for source in self.by_source:
            if source in name:
                return source
        return None

    def on_diagnostics(self, message):
        now = time.time()
        observed = set()
        for status in message.status:
            source = self.source_from_name(status.name)
            if source is None:
                continue
            code = self.by_source[source]["code"]
            observed.add(code)
            if status.level >= DiagnosticStatus.WARN:
                self.store.observe(code, now, self.freeze_frame(status))
            else:
                self.store.clear(code, now)
        self.seen_in_cycle.update(observed)

    def on_odometry(self, message):
        self.latest["speed_mps"] = float(message.twist.twist.linear.x)

    def on_actuation(self, message):
        self.latest.update(steer=float(message.steer), throttle=float(message.throttle),
                           brake=float(message.brake))

    def on_gate(self, message):
        self.latest["control_source"] = int(message.selected_source)

    def on_safety(self, message):
        self.latest["safety_level"] = int(message.overall)

    def on_mcu_status(self, message):
        """MCU fault_code (FC_* bitmask from 0x203) → persistent DTC records."""
        self.latest.update(mcu_state=int(message.system_state),
                           mcu_active_source=int(message.active_source),
                           mcu_degrade_reason=message.degrade_reason)
        now = time.time()
        freeze = dict(self.latest,
                      fault_code=int(message.fault_code),
                      heartbeat_age_s=float(message.heartbeat_age_s),
                      command_age_s=float(message.command_age_s),
                      protocol_version_ok=bool(message.protocol_version_ok))
        for bit, code in self.mcu_faults:
            if message.fault_code & (1 << bit):
                self.store.observe(code, now, freeze)
            else:
                self.store.clear(code, now)

    def flush_and_publish(self):
        persistence_error = None
        try:
            self.store.flush()
            message = String()
            message.data = json.dumps(self.store.snapshot(), ensure_ascii=False,
                                      separators=(",", ":"))
            self.publisher.publish(message)
        except OSError as error:
            persistence_error = error
            self.get_logger().error(f"DTC persistence failed: {error}")
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "dtc_recorder: persistence"
        status.hardware_id = "soc-filesystem"
        status.level = (DiagnosticStatus.OK if persistence_error is None
                        else DiagnosticStatus.ERROR)
        status.message = ("DTC persistence healthy" if persistence_error is None
                          else str(persistence_error))
        diagnostic.status = [status]
        self.diagnostic_publisher.publish(diagnostic)


def main(args=None):
    rclpy.init(args=args)
    node = DtcRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.store.flush()
        finally:
            node.destroy_node()
            rclpy.shutdown()
