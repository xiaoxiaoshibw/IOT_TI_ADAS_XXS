#!/usr/bin/env python3
"""Execute the MCU test-build fault matrix through Linux SocketCAN.

The runner refuses to inject until CAN ID 0x204 proves ADAS_TEST_BUILD=1 and
protocol version 3. Each case is cleared before and after execution. Evidence
is written as JSON for the safety traceability package.
"""

import argparse
import json
import pathlib
import socket
import struct
import time


CAN_FRAME = struct.Struct("=IB3x8s")
CANID_MCU_CONTROL = 0x201
CANID_MCU_HEARTBEAT = 0x202
CANID_MCU_E2E_DIAG = 0x204
CANID_FAULT_INJECT = 0x301
PROTOCOL_VERSION = 3  # HIL 默认 protocol v3；MCU 上报必须匹配此常量
INJ_CLEAR = 0


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x31) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def protect(can_id, payload):
    if len(payload) != 8:
        raise ValueError("CAN payload must be exactly 8 bytes")
    result = bytearray(payload)
    result[7] = crc8(bytes((can_id & 0xFF, (can_id >> 8) & 0xFF)) + result[:7])
    return bytes(result)


def valid(can_id, payload):
    return len(payload) == 8 and protect(can_id, payload) == payload


def injection_frame(command, sequence):
    payload = bytearray(8)
    payload[0] = command & 0xFF
    payload[5] = sequence & 0xFF
    return protect(CANID_FAULT_INJECT, payload)


class CanSession:
    def __init__(self, interface):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.settimeout(0.05)

    def close(self):
        self.sock.close()

    def send(self, can_id, payload):
        self.sock.send(CAN_FRAME.pack(can_id, 8, payload))

    def receive_until(self, wanted_ids, deadline):
        frames = {}
        while time.monotonic() < deadline:
            try:
                raw = self.sock.recv(CAN_FRAME.size)
            except TimeoutError:
                continue
            can_id, dlc, payload = CAN_FRAME.unpack(raw)
            can_id &= 0x7FF
            if dlc == 8 and can_id in wanted_ids and valid(can_id, payload):
                frames[can_id] = payload
        return frames


def require_test_build(session):
    frames = session.receive_until(
        {CANID_MCU_E2E_DIAG, CANID_MCU_HEARTBEAT}, time.monotonic() + 1.5)
    diagnostic = frames.get(CANID_MCU_E2E_DIAG)
    heartbeat = frames.get(CANID_MCU_HEARTBEAT)
    if diagnostic is None:
        raise RuntimeError("MCU E2E diagnostic 0x204 not received")
    if diagnostic[6] != PROTOCOL_VERSION:
        raise RuntimeError(f"protocol mismatch: MCU={diagnostic[6]}, runner={PROTOCOL_VERSION}")
    if diagnostic[5] & 0x01 == 0:
        raise RuntimeError("refusing injection: MCU reports production build")
    if heartbeat is None:
        raise RuntimeError("MCU heartbeat 0x202 not received")
    if heartbeat[0] != 2:
        raise RuntimeError(f"fault matrix requires ACTIVE baseline, state={heartbeat[0]}")
    if heartbeat[2] & 0x03 != 0x03:
        raise RuntimeError("fault matrix requires both primary and backup sources fresh")


def execute_case(session, case, sequence):
    session.send(CANID_FAULT_INJECT, injection_frame(INJ_CLEAR, sequence))
    time.sleep(0.15)
    sequence = (sequence + 1) & 0xFF
    started = time.monotonic()
    session.send(CANID_FAULT_INJECT, injection_frame(case["command"], sequence))
    # capture_s can be raised per case for slew-limited outputs (FAILSAFE/MRM
    # ramp their deceleration instead of stepping to full brake).
    frames = session.receive_until(
        {CANID_MCU_CONTROL, CANID_MCU_HEARTBEAT}, started + case.get("capture_s", 0.25))
    heartbeat = frames.get(CANID_MCU_HEARTBEAT)
    control = frames.get(CANID_MCU_CONTROL)
    failures = []
    if heartbeat is None:
        failures.append("0x202 heartbeat missing")
    elif heartbeat[0] != case["expected_state"]:
        failures.append(
            f"state {heartbeat[0]} != expected {case['expected_state']}")
    if control is None:
        failures.append("0x201 control feedback missing")
    else:
        if case["require_full_brake"] and (control[4] != 0 or control[5] != 100):
            failures.append(
                f"safe output mismatch: throttle={control[4]} brake={control[5]}")
        if case.get("require_zero_throttle") and control[4] != 0:
            failures.append(f"throttle not zero: throttle={control[4]}")
    elapsed_ms = (time.monotonic() - started) * 1000.0
    sequence = (sequence + 1) & 0xFF
    session.send(CANID_FAULT_INJECT, injection_frame(INJ_CLEAR, sequence))
    return {
        "id": case["id"],
        "name": case["name"],
        "verdict": "PASS" if not failures else "FAIL",
        "elapsed_capture_ms": elapsed_ms,
        "observed_state": None if heartbeat is None else heartbeat[0],
        "observed_throttle_pct": None if control is None else control[4],
        "observed_brake_pct": None if control is None else control[5],
        "failures": failures,
    }, sequence


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="can1")  # Jetson Orin Nano HIL: PEAK PCAN-USB → SocketCAN can1
    parser.add_argument("--matrix", type=pathlib.Path,
                        default=pathlib.Path(__file__).with_name("fault_matrix.json"))
    parser.add_argument("--output", type=pathlib.Path,
                        default=pathlib.Path("test/hil/evidence/mcu_fault_matrix.json"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    matrix = json.loads(args.matrix.read_text(encoding="utf-8"))
    if matrix.get("schema_version") != 1 or matrix.get("protocol_version") != PROTOCOL_VERSION:
        raise SystemExit("unsupported fault-matrix schema or protocol")
    if args.dry_run:
        print(json.dumps({case["id"]: injection_frame(case["command"], index).hex()
                          for index, case in enumerate(matrix["cases"])}, indent=2))
        return

    session = CanSession(args.interface)
    results = []
    sequence = 0
    try:
        require_test_build(session)
        for case in matrix["cases"]:
            result, sequence = execute_case(session, case, sequence)
            results.append(result)
    finally:
        session.close()
    report = {
        "schema_version": 1,
        "interface": args.interface,
        "timestamp_unix": time.time(),
        "verdict": "PASS" if all(item["verdict"] == "PASS" for item in results) else "FAIL",
        "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if report["verdict"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
