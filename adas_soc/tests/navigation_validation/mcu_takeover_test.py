#!/usr/bin/env python3
"""Observe MCU takeover, or invoke the existing guarded test-build fault matrix.

Production firmware is observation-only.  Active fault injection is delegated
to test/hil/run_mcu_fault_matrix.py, which requires CAN 0x204 proof of a test
build and protocol v3 before transmitting any injection command.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import rclpy
from adas_msgs.msg import McuStatus
from rclpy.node import Node

from ros_validation import reliable_qos, write_result

__test__ = False


class TakeoverObserver(Node):
    def __init__(self, expected_source: int) -> None:
        super().__init__("phase3_mcu_takeover_test")
        self.expected_source = expected_source
        self.baseline_wall = None
        self.fault_wall = None
        self.takeover_wall = None
        self.states = []
        self.latest = None
        self.create_subscription(McuStatus, "/adas/mcu/status", self._on_status, reliable_qos())

    def _on_status(self, msg: McuStatus) -> None:
        now = time.time()
        self.latest = msg
        self.states.append(
            {
                "timestamp": now,
                "system_state": int(msg.system_state),
                "active_source": int(msg.active_source),
                "primary_fresh": bool(msg.primary_fresh),
                "fault_code": int(msg.fault_code),
                "heartbeat_age_s": float(msg.heartbeat_age_s),
                "crc_error_count": int(msg.crc_error_count),
            }
        )
        if msg.system_state == McuStatus.SYS_ACTIVE and msg.active_source == McuStatus.SRC_PRIMARY:
            self.baseline_wall = self.baseline_wall or now
        if self.baseline_wall and self.fault_wall is None and (
            not msg.primary_fresh or msg.system_state != McuStatus.SYS_ACTIVE
        ):
            self.fault_wall = now
        if self.fault_wall and self.takeover_wall is None and msg.active_source == self.expected_source:
            self.takeover_wall = now


def observe(output: Path, timeout: float, expected_source: int) -> int:
    rclpy.init()
    node = TakeoverObserver(expected_source)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and node.takeover_wall is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        passed = node.baseline_wall is not None and node.fault_wall is not None and node.takeover_wall is not None
        result = {
            "test": "F280025C takeover observation",
            "passed": passed,
            "baseline_timestamp": node.baseline_wall,
            "fault_timestamp": node.fault_wall,
            "takeover_timestamp": node.takeover_wall,
            "takeover_latency_s": (
                node.takeover_wall - node.fault_wall if node.takeover_wall and node.fault_wall else None
            ),
            "expected_active_source": expected_source,
            "samples": node.states,
        }
        if not passed:
            result["error"] = "baseline/fault/takeover sequence was not observed before timeout"
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return write_result(output, result)


def run_matrix(output: Path, runner: Path, matrix: Path, interface: str) -> int:
    matrix_output = output.with_name("mcu_fault_matrix_raw.json")
    command = [
        str(runner),
        "--interface",
        interface,
        "--matrix",
        str(matrix),
        "--output",
        str(matrix_output),
    ]
    completed = subprocess.run(command, check=False, text=True, capture_output=True)
    result = {
        "test": "guarded MCU test-build fault matrix",
        "passed": completed.returncode == 0,
        "command": command,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
        "raw_result": str(matrix_output),
        "production_firmware_injection_allowed": False,
    }
    if matrix_output.exists():
        result["matrix_result"] = json.loads(matrix_output.read_text(encoding="utf-8"))
    else:
        result["error"] = "runner refused injection or produced no evidence"
    return write_result(output, result)


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mode", choices=("observe", "test-build-matrix"), default="observe")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--expected-source", type=int, default=McuStatus.SRC_BACKUP)
    parser.add_argument("--interface", default="can1")
    parser.add_argument("--runner", type=Path, default=root / "test/hil/run_mcu_fault_matrix.py")
    parser.add_argument("--matrix", type=Path, default=root / "test/hil/fault_matrix.json")
    args = parser.parse_args()
    if args.mode == "observe":
        return observe(args.output, args.timeout, args.expected_source)
    return run_matrix(args.output, args.runner, args.matrix, args.interface)


if __name__ == "__main__":
    raise SystemExit(main())

