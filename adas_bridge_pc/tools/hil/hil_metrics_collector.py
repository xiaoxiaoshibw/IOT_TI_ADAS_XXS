#!/usr/bin/env python3
"""Collect read-only metrics for a running ADAS HIL session.

CAN feedback and interface counters are observed on Jetson over SSH.  CARLA is
observed over RPC.  This process never opens the CANalyst transmit channel and
never calls ``apply_control``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import statistics
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any


CANDUMP_RE = re.compile(
    r"^\((?P<timestamp>\d+(?:\.\d+)?)\)\s+\S+\s+"
    r"(?P<can_id>[0-9A-Fa-f]{3})#(?P<payload>[0-9A-Fa-f]{16})$"
)
STATE_RE = re.compile(r"\bcan state\s+([A-Z-]+)")
BERR_RE = re.compile(r"berr-counter\s+tx\s+(\d+)\s+rx\s+(\d+)")
COUNTER_ROW_RE = re.compile(
    r"^\s*(RX|TX):\s+bytes\s+packets\s+errors\s+dropped[^\n]*\n"
    r"\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", re.MULTILINE
)
BUS_ERROR_RE = re.compile(
    r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass\s+bus-off\n"
    r"\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", re.MULTILINE
)


def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (((value << 1) ^ 0x31) if value & 0x80 else value << 1) & 0xFF
    return value


def frame_crc(can_id: int, data: bytes) -> int:
    return crc8(bytes((can_id & 0xFF, (can_id >> 8) & 0xFF)) + data[:7])


def decode_feedback(can_id: int, payload: str) -> dict[str, Any]:
    data = bytes.fromhex(payload)
    if len(data) != 8 or data[7] != frame_crc(can_id, data):
        raise ValueError("invalid CRC or payload length")
    if can_id == 0x201:
        steer_raw, accel_raw = struct.unpack_from("<hh", data)
        if data[4] > 100 or data[5] > 100 or (data[4] and data[5]):
            raise ValueError("invalid control payload")
        return {
            "kind": "control",
            "steering": steer_raw * 0.01 / 30.0,
            "acceleration_mps2": accel_raw * 0.001,
            "throttle": data[4] * 0.01,
            "brake": data[5] * 0.01,
            "sequence": data[6],
        }
    if can_id == 0x202:
        return {
            "kind": "heartbeat",
            "state": data[0],
            "active_source": data[1],
            "heartbeat_sequence": data[4],
        }
    if can_id == 0x203:
        return {
            "kind": "diagnostic",
            "fault_code": data[0] | (data[1] << 8),
            "reset_reason": data[2],
            "primary_timeout_count": data[3],
            "crc_error_count": data[5],
            "loop_overrun_count": data[6],
        }
    if can_id == 0x204:
        return {"kind": "e2e", "can_recovery_count": data[3]}
    raise ValueError("unsupported CAN ID")


def parse_ip_link_stats(text: str) -> dict[str, Any]:
    rows: dict[str, tuple[int, int, int]] = {}
    for match in COUNTER_ROW_RE.finditer(text):
        rows[match.group(1)] = (
            int(match.group(3)), int(match.group(4)), int(match.group(5))
        )
    if "RX" not in rows or "TX" not in rows:
        raise ValueError("missing RX/TX counter rows")
    rx_packets, rx_errors, rx_drop = rows["RX"]
    tx_packets, tx_errors, tx_drop = rows["TX"]
    berr = BERR_RE.search(text)
    bus_error = BUS_ERROR_RE.search(text)
    state = STATE_RE.search(text)
    return {
        "rx_count": rx_packets,
        "tx_count": tx_packets,
        "error": rx_errors + tx_errors + (int(bus_error.group(2)) if bus_error else 0),
        "drop": rx_drop + tx_drop,
        "bus_error": int(bus_error.group(2)) if bus_error else 0,
        "bus_off": int(bus_error.group(6)) if bus_error else 0,
        "berr_tx": int(berr.group(1)) if berr else -1,
        "berr_rx": int(berr.group(2)) if berr else -1,
        "can_state": state.group(1) if state else "UNKNOWN",
    }


class RemoteCanCollector:
    def __init__(self, host: str, output_dir: Path):
        self.host = host
        self.output_dir = output_dir
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.state: int | None = None
        self.active_source: int | None = None
        self.fault_code: int | None = None
        self.reset_reason: int | None = None
        self.primary_timeout_count: int | None = None
        self.crc_error_count: int | None = None
        self.loop_overrun_count: int | None = None
        self.can_recovery_count: int | None = None
        self.invalid_frames = 0
        self.control_frames = 0
        self.stats_samples = 0
        self.processes: list[subprocess.Popen[str]] = []
        self.threads: list[threading.Thread] = []

    def start(self) -> None:
        frame_command = (
            "exec stdbuf -oL candump -L "
            "can1,201:7FF,202:7FF,203:7FF,204:7FF"
        )
        stats_command = (
            "trap 'exit 0' HUP INT TERM; "
            "while :; do printf '__SAMPLE__ %s\\n' \"$(date +%s.%N)\"; "
            "ip -details -statistics link show can1; printf '__END__\\n'; sleep 1; done"
        )
        frames = subprocess.Popen(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", self.host,
             frame_command],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        stats = subprocess.Popen(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", self.host,
             stats_command],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        self.processes = [frames, stats]
        self.threads = [
            threading.Thread(target=self._read_frames, args=(frames,), daemon=True),
            threading.Thread(target=self._read_stats, args=(stats,), daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def _read_frames(self, process: subprocess.Popen[str]) -> None:
        path = self.output_dir / "mcu_control.csv"
        raw_path = self.output_dir / "candump.log"
        with (path.open("w", newline="", encoding="utf-8") as stream,
              raw_path.open("w", encoding="utf-8") as raw_stream):
            writer = csv.writer(stream)
            writer.writerow([
                "timestamp", "sequence", "steering", "acceleration_mps2",
                "throttle", "brake", "state", "active_source", "fault_code",
                "reset_reason", "primary_timeout_count", "crc_error_count",
                "loop_overrun_count", "can_recovery_count",
            ])
            assert process.stdout is not None
            for raw_line in process.stdout:
                if self.stop_event.is_set():
                    break
                raw_stream.write(raw_line)
                raw_stream.flush()
                match = CANDUMP_RE.match(raw_line.strip())
                if not match:
                    continue
                can_id = int(match.group("can_id"), 16)
                try:
                    decoded = decode_feedback(can_id, match.group("payload"))
                except ValueError:
                    with self.lock:
                        self.invalid_frames += 1
                    continue
                with self.lock:
                    if decoded["kind"] == "heartbeat":
                        self.state = decoded["state"]
                        self.active_source = decoded["active_source"]
                        continue
                    if decoded["kind"] == "diagnostic":
                        self.fault_code = decoded["fault_code"]
                        self.reset_reason = decoded["reset_reason"]
                        self.primary_timeout_count = decoded["primary_timeout_count"]
                        self.crc_error_count = decoded["crc_error_count"]
                        self.loop_overrun_count = decoded["loop_overrun_count"]
                        continue
                    if decoded["kind"] == "e2e":
                        self.can_recovery_count = decoded["can_recovery_count"]
                        continue
                    state = self.state
                    source = self.active_source
                    diagnostics = (
                        self.fault_code, self.reset_reason, self.primary_timeout_count,
                        self.crc_error_count, self.loop_overrun_count,
                        self.can_recovery_count,
                    )
                    self.control_frames += 1
                writer.writerow([
                    match.group("timestamp"), decoded["sequence"],
                    f'{decoded["steering"]:.6f}',
                    f'{decoded["acceleration_mps2"]:.6f}',
                    f'{decoded["throttle"]:.6f}', f'{decoded["brake"]:.6f}',
                    "" if state is None else state, "" if source is None else source,
                    *("" if value is None else value for value in diagnostics),
                ])
                if self.control_frames % 100 == 0:
                    stream.flush()

    def _read_stats(self, process: subprocess.Popen[str]) -> None:
        path = self.output_dir / "can_metrics.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            fieldnames = [
                "time", "rx_count", "tx_count", "error", "drop", "bus_error",
                "bus_off", "berr_tx", "berr_rx", "can_state",
            ]
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            timestamp = ""
            block: list[str] = []
            assert process.stdout is not None
            for raw_line in process.stdout:
                if self.stop_event.is_set():
                    break
                line = raw_line.rstrip("\n")
                if line.startswith("__SAMPLE__ "):
                    timestamp = line.split(maxsplit=1)[1]
                    block = []
                elif line == "__END__":
                    try:
                        row = parse_ip_link_stats("\n".join(block))
                    except ValueError:
                        continue
                    writer.writerow({"time": timestamp, **row})
                    stream.flush()
                    with self.lock:
                        self.stats_samples += 1
                else:
                    block.append(line)

    def health(self) -> tuple[bool, str]:
        for process, name in zip(self.processes, ("candump", "can_stats")):
            status = process.poll()
            if status is not None and not self.stop_event.is_set():
                stderr = ""
                if process.stderr is not None:
                    stderr = process.stderr.read().strip()
                return False, f"remote {name} exited rc={status}: {stderr}"
        return True, ""

    def stop(self) -> None:
        self.stop_event.set()
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        for thread in self.threads:
            thread.join(timeout=2)


def find_ego(world: Any, timeout_s: float = 10.0) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        actors = world.get_actors().filter("vehicle.*")
        for actor in actors:
            if actor.attributes.get("role_name", "") == "hero":
                return actor
        time.sleep(0.2)
    raise RuntimeError("CARLA hero vehicle not found")


def process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except (ProcessLookupError, PermissionError):
        return False
    return True


def write_timing(control_path: Path, timing_path: Path) -> dict[str, float | int]:
    timestamps: list[float] = []
    sequences: list[int] = []
    states: list[int] = []
    row_states: list[str] = []
    row_sources: list[str] = []
    diagnostic_fields = (
        "fault_code", "reset_reason", "primary_timeout_count", "crc_error_count",
        "loop_overrun_count", "can_recovery_count",
    )
    diagnostic_values: dict[str, list[int]] = {field: [] for field in diagnostic_fields}
    with control_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            timestamps.append(float(row["timestamp"]))
            sequences.append(int(row["sequence"]))
            row_states.append(row["state"])
            row_sources.append(row["active_source"])
            for field in diagnostic_fields:
                value = row.get(field, "")
                if value:
                    diagnostic_values[field].append(int(value))
            if row["state"]:
                states.append(int(row["state"]))
    periods = [(b - a) * 1000.0 for a, b in zip(timestamps, timestamps[1:])]
    jitters = [period - 10.0 for period in periods]
    running_count = 0
    running_sum = 0.0
    with timing_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "timestamp", "period_ms", "frequency_hz", "jitter_ms",
            "running_average_ms", "running_max_ms", "running_jitter_rms_ms",
        ])
        square_sum = 0.0
        current_max = 0.0
        for timestamp, period, jitter in zip(timestamps[1:], periods, jitters):
            running_count += 1
            running_sum += period
            square_sum += jitter * jitter
            current_max = max(current_max, period)
            writer.writerow([
                f"{timestamp:.6f}", f"{period:.6f}",
                f"{(1000.0 / period) if period > 0 else 0.0:.6f}",
                f"{jitter:.6f}", f"{running_sum / running_count:.6f}",
                f"{current_max:.6f}", f"{math.sqrt(square_sum / running_count):.6f}",
            ])
    anomaly_path = timing_path.with_name("mcu_sequence_anomalies.csv")
    with anomaly_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "previous_timestamp", "current_timestamp", "gap_ms",
            "previous_sequence", "current_sequence", "sequence_delta",
            "state", "active_source",
        ])
        for index, (period, previous, current) in enumerate(
                zip(periods, sequences, sequences[1:]), start=1):
            delta = (current - previous) & 0xFF
            if period > 30.0 or delta != 1:
                writer.writerow([
                    f"{timestamps[index - 1]:.6f}", f"{timestamps[index]:.6f}",
                    f"{period:.6f}", previous, current, delta,
                    row_states[index], row_sources[index],
                ])
    non_active_intervals: list[tuple[float, float, int, str]] = []
    interval_start: int | None = None
    for index, state in enumerate(row_states):
        is_non_active = bool(state) and state != "2"
        if is_non_active and interval_start is None:
            interval_start = index
        if interval_start is not None and (not is_non_active or index == len(row_states) - 1):
            interval_end = index if is_non_active and index == len(row_states) - 1 else index - 1
            interval_states = "/".join(sorted(set(row_states[interval_start:interval_end + 1])))
            non_active_intervals.append((
                timestamps[interval_start], timestamps[interval_end],
                interval_end - interval_start + 1, interval_states,
            ))
            interval_start = None
    interval_path = timing_path.with_name("mcu_non_active_intervals.csv")
    with interval_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["start_timestamp", "end_timestamp", "duration_s", "samples", "states"])
        for start, end, samples, interval_states in non_active_intervals:
            writer.writerow([
                f"{start:.6f}", f"{end:.6f}", f"{end - start:.6f}", samples,
                interval_states,
            ])
    seq_stalls = sum(a == b for a, b in zip(sequences, sequences[1:]))
    seq_gaps = sum(((b - a) & 0xFF) != 1 for a, b in zip(sequences, sequences[1:]))
    diagnostic_metrics: dict[str, Any] = {
        "mcu_diagnostics_available": all(diagnostic_values.values()),
    }
    for field, values in diagnostic_values.items():
        diagnostic_metrics[f"{field}_start"] = values[0] if values else None
        diagnostic_metrics[f"{field}_end"] = values[-1] if values else None
        diagnostic_metrics[f"{field}_delta"] = values[-1] - values[0] if values else None
    reset_values = diagnostic_values["reset_reason"]
    diagnostic_metrics["reset_reason_changes"] = sum(
        previous != current for previous, current in zip(reset_values, reset_values[1:])
    )
    return {
        "control_frames": len(timestamps),
        "control_duration_s": timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0,
        "control_frequency_hz": ((len(timestamps) - 1) / (timestamps[-1] - timestamps[0]))
        if len(timestamps) > 1 and timestamps[-1] > timestamps[0] else 0.0,
        "period_average_ms": statistics.fmean(periods) if periods else 0.0,
        "period_max_ms": max(periods, default=0.0),
        "jitter_rms_ms": math.sqrt(statistics.fmean([value * value for value in jitters]))
        if jitters else 0.0,
        "jitter_max_abs_ms": max((abs(value) for value in jitters), default=0.0),
        "control_outage_count": sum(period > 30.0 for period in periods),
        "control_outage_max_s": max((period for period in periods if period > 30.0), default=0.0) / 1000.0,
        "control_outage_excess_total_s": sum(
            period - 10.0 for period in periods if period > 30.0
        ) / 1000.0,
        "sequence_stalls": seq_stalls,
        "sequence_gaps": seq_gaps,
        "mcu_non_active_samples": sum(state != 2 for state in states),
        "mcu_state_samples": len(states),
        "mcu_non_active_interval_count": len(non_active_intervals),
        "mcu_non_active_interval_max_s": max(
            (end - start for start, end, _samples, _states in non_active_intervals),
            default=0.0,
        ),
        **diagnostic_metrics,
    }


def counter_summary(path: Path) -> dict[str, Any]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    result: dict[str, Any] = {"can_samples": len(rows)}
    for field in ("rx_count", "tx_count", "error", "drop", "bus_error", "bus_off"):
        available = bool(rows) and field in rows[0]
        values = [int(row[field]) for row in rows] if available else []
        result[f"can_{field}_start"] = values[0] if values else None
        result[f"can_{field}_end"] = values[-1] if values else None
        result[f"can_{field}_delta"] = (
            values[-1] - values[0] if len(values) > 1 else (0 if values else None)
        )
    result["can_states"] = sorted({row["can_state"] for row in rows})
    result["berr_tx_max"] = max((int(row["berr_tx"]) for row in rows), default=-1)
    result["berr_rx_max"] = max((int(row["berr_rx"]) for row in rows), default=-1)
    return result


def vehicle_summary(path: Path) -> dict[str, Any]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return {"vehicle_samples": 0, "vehicle_duration_s": 0.0}
    first, last = rows[0], rows[-1]
    dx = float(last["x"]) - float(first["x"])
    dy = float(last["y"]) - float(first["y"])
    speeds = [float(row["speed"]) for row in rows]
    return {
        "vehicle_samples": len(rows),
        "vehicle_duration_s": float(last["timestamp"]) - float(first["timestamp"]),
        "vehicle_displacement_m": math.hypot(dx, dy),
        "vehicle_speed_min_mps": min(speeds),
        "vehicle_speed_max_mps": max(speeds),
        "vehicle_speed_average_mps": statistics.fmean(speeds),
    }


def write_summary(output_dir: Path, requested_s: float, metrics: dict[str, Any]) -> None:
    passed = all([
        metrics["control_duration_s"] >= max(0.0, requested_s - 2.0),
        metrics["control_frequency_hz"] >= 95.0,
        metrics["sequence_stalls"] == 0,
        metrics["can_error_delta"] == 0,
        metrics["can_drop_delta"] == 0,
        metrics["can_bus_error_delta"] == 0,
        metrics["can_bus_off_delta"] in (None, 0),
        metrics["mcu_non_active_samples"] == 0,
        metrics["vehicle_samples"] > 0,
        not metrics.get("mcu_diagnostics_available") or all(
            metrics.get(field) == 0 for field in (
                "crc_error_count_delta", "loop_overrun_count_delta",
                "can_recovery_count_delta", "reset_reason_changes",
            )
        ),
    ])
    metrics["phase1_pass"] = passed
    (output_dir / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    lines = [
        "# HIL 长时间稳定性测试摘要",
        "",
        f"- 结果：{'PASS' if passed else 'FAIL'}",
        f"- 请求运行时间：{requested_s:.1f} s",
        f"- 0x201 有效采集时长：{metrics['control_duration_s']:.3f} s",
        f"- 0x201 帧数 / 频率：{metrics['control_frames']} / {metrics['control_frequency_hz']:.3f} Hz",
        f"- 平均周期 / 最大周期：{metrics['period_average_ms']:.4f} / {metrics['period_max_ms']:.4f} ms",
        f"- 周期抖动 RMS / 最大绝对抖动：{metrics['jitter_rms_ms']:.4f} / {metrics['jitter_max_abs_ms']:.4f} ms",
        f"- 序列停滞 / 非连续：{metrics['sequence_stalls']} / {metrics['sequence_gaps']}",
        f"- 控制空窗次数 / 最长空窗：{metrics['control_outage_count']} / {metrics['control_outage_max_s']:.3f} s",
        f"- 超出标称 10 ms 的空窗累计：{metrics['control_outage_excess_total_s']:.3f} s",
        f"- CAN RX/TX 增量：{metrics['can_rx_count_delta']} / {metrics['can_tx_count_delta']}",
        f"- CAN error/drop/bus-error/bus-off 增量：{metrics['can_error_delta']} / {metrics['can_drop_delta']} / {metrics['can_bus_error_delta']} / "
        f"{metrics['can_bus_off_delta'] if metrics['can_bus_off_delta'] is not None else 'N/A'}",
        f"- CAN 状态：{', '.join(metrics['can_states'])}",
        f"- MCU 非 ACTIVE 样本：{metrics['mcu_non_active_samples']} / {metrics['mcu_state_samples']}",
        f"- MCU 非 ACTIVE 区间数 / 最长：{metrics['mcu_non_active_interval_count']} / {metrics['mcu_non_active_interval_max_s']:.3f} s",
        f"- 车辆采样时长 / 位移：{metrics['vehicle_duration_s']:.3f} s / {metrics.get('vehicle_displacement_m', 0.0):.3f} m",
        f"- 车辆速度 min/avg/max：{metrics.get('vehicle_speed_min_mps', 0.0):.3f} / {metrics.get('vehicle_speed_average_mps', 0.0):.3f} / {metrics.get('vehicle_speed_max_mps', 0.0):.3f} m/s",
        "",
        "本摘要仅由同目录真实 CSV 自动计算。科研绘图与正式竞赛报告属于 Phase 2。",
    ]
    if metrics.get("mcu_diagnostics_available"):
        lines.insert(-2,
            "- MCU reset reason 变化 / CRC / loop overrun / CAN recovery 增量："
            f"{metrics['reset_reason_changes']} / {metrics['crc_error_count_delta']} / "
            f"{metrics['loop_overrun_count_delta']} / {metrics['can_recovery_count_delta']}"
        )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--jetson-host", default="jetson@192.168.100.32")
    parser.add_argument("--manager-pid", type=int, required=True)
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--sample-period", type=float, default=0.1)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    if not math.isfinite(args.sample_period) or args.sample_period <= 0:
        parser.error("--sample-period must be finite and positive")
    return args


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    try:
        import carla  # type: ignore
    except ImportError as exc:
        print(f"FAIL CARLA Python API import: {exc}", file=sys.stderr)
        return 2

    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(3.0)
    world = client.get_world()
    ego = find_ego(world)
    remote = RemoteCanCollector(args.jetson_host, args.output_dir)
    remote.start()
    start = time.monotonic()
    deadline = start + args.duration
    next_sample = start
    next_progress = start + 30.0
    interrupted = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    vehicle_path = args.output_dir / "vehicle_state.csv"
    collector_error = ""
    try:
        with vehicle_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["timestamp", "speed", "x", "y", "yaw"])
            while time.monotonic() < deadline and not interrupted:
                if not process_alive(args.manager_pid):
                    raise RuntimeError(f"HIL manager PID {args.manager_pid} exited early")
                healthy, reason = remote.health()
                if not healthy:
                    raise RuntimeError(reason)
                now = time.monotonic()
                if now >= next_sample:
                    transform = ego.get_transform()
                    velocity = ego.get_velocity()
                    speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
                    writer.writerow([
                        f"{time.time():.6f}", f"{speed:.6f}",
                        f"{transform.location.x:.6f}", f"{transform.location.y:.6f}",
                        f"{transform.rotation.yaw:.6f}",
                    ])
                    stream.flush()
                    next_sample += args.sample_period
                if now >= next_progress:
                    elapsed = now - start
                    print(
                        f"PROGRESS elapsed={elapsed:.1f}s/"
                        f"{args.duration:.1f}s control_frames={remote.control_frames} "
                        f"can_samples={remote.stats_samples}", flush=True
                    )
                    next_progress += 30.0
                time.sleep(min(0.02, max(0.001, next_sample - time.monotonic())))
    except Exception as exc:  # retain partial CSV and make failure explicit
        collector_error = str(exc)
    finally:
        remote.stop()

    if interrupted:
        print("FAIL metric collection interrupted", file=sys.stderr)
        return 130
    if collector_error:
        print(f"FAIL metric collection: {collector_error}", file=sys.stderr)
        return 3
    if remote.invalid_frames:
        print(f"FAIL invalid MCU feedback frames={remote.invalid_frames}", file=sys.stderr)
        return 4

    timing = write_timing(args.output_dir / "mcu_control.csv", args.output_dir / "mcu_timing.csv")
    metrics = {
        "requested_duration_s": args.duration,
        "collector_wall_duration_s": time.monotonic() - start,
        "invalid_feedback_frames": remote.invalid_frames,
        **timing,
        **counter_summary(args.output_dir / "can_metrics.csv"),
        **vehicle_summary(vehicle_path),
    }
    write_summary(args.output_dir, args.duration, metrics)
    if not metrics.get("control_frames") or not metrics.get("can_samples"):
        print("FAIL required CAN samples are empty", file=sys.stderr)
        return 5
    print(f"PASS metrics collected in {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
