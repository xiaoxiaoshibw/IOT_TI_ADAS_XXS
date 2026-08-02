#!/usr/bin/env python3
"""Terminal safety HMI for the ADAS HIL platform.

Subscribes to /adas/mcu/status (adas_msgs/McuStatus) and
/adas/diagnostics/dtc_history, then renders the P0 visualization set:
control source, safety state, fault code, last-valid-command age and the
degrade reason. Formatting is ROS-free so it can be unit-tested on any host.

Run on the Orin (or any host that sees the DDS domain):

    python3 SoC/tools/safety_hmi.py
"""

import argparse
import json
import sys
import time


STATE_NAMES = {
    0: "INIT", 1: "STANDBY", 2: "ACTIVE", 3: "DEGRADED",
    4: "MRM", 5: "EMERGENCY_BRAKE", 6: "FAILSAFE", 7: "FAULT_LOCK",
}
# 高于 DEGRADED 的状态用红色强调；DEGRADED/MRM 黄色；正常绿色。
ALERT_STATES = {5, 6, 7}
WARN_STATES = {3, 4}
SOURCE_NAMES = {0: "NONE", 1: "PRIMARY", 2: "BACKUP", 9: "MCU_WATCHDOG"}

ANSI_RED = "\x1b[31;1m"
ANSI_YELLOW = "\x1b[33;1m"
ANSI_GREEN = "\x1b[32m"
ANSI_RESET = "\x1b[0m"


def state_name(state):
    return STATE_NAMES.get(state, f"UNKNOWN({state})")


def source_name(source):
    return SOURCE_NAMES.get(source, f"UNKNOWN({source})")


def format_age(age_s):
    """Negative means never received — that is itself a safety-relevant fact."""
    if age_s is None or age_s < 0.0:
        return "never"
    if age_s < 1.0:
        return f"{age_s * 1000.0:.0f} ms"
    return f"{age_s:.1f} s"


def render_status(status, colors=False):
    """Render one McuStatus-like dict into terminal lines.

    `status` uses the field names of adas_msgs/McuStatus; missing fields
    are rendered as stale/never rather than raising, because an incomplete
    view must still show that evidence is missing.
    """
    state = status.get("system_state")
    red, yellow, green, reset = (
        (ANSI_RED, ANSI_YELLOW, ANSI_GREEN, ANSI_RESET) if colors
        else ("", "", "", ""))
    if state in ALERT_STATES:
        color = red
    elif state in WARN_STATES or state is None:
        color = yellow
    else:
        color = green

    heartbeat_age = status.get("heartbeat_age_s")
    stale = heartbeat_age is None or heartbeat_age < 0.0 or heartbeat_age > 0.5
    header = f"{color}MCU {state_name(state) if state is not None else 'NO DATA'}{reset}"
    if stale:
        header += f" {red}[HEARTBEAT STALE]{reset}"

    lines = [
        header,
        f"  control source : {source_name(status.get('active_source', 0))}"
        f"  (primary_fresh={status.get('primary_fresh', False)}"
        f" backup_fresh={status.get('backup_fresh', False)})",
        f"  fault code     : 0x{status.get('fault_code', 0):04X}"
        f"  level={status.get('fault_level', 0)}",
        f"  degrade reason : {status.get('degrade_reason') or 'none'}",
        f"  command age    : {format_age(status.get('command_age_s'))}"
        f"   heartbeat age: {format_age(heartbeat_age)}"
        f"   feedback age: {format_age(status.get('feedback_age_s'))}",
        f"  protocol       : v{status.get('protocol_version', 0)}"
        f" ok={status.get('protocol_version_ok', False)}"
        f" test_build={status.get('test_build', False)}",
    ]
    flags = [name for name, active in (
        ("AEB_FLOOR", status.get("aeb_floor_active")),
        ("DEGRADED", status.get("degraded")),
        ("ESTOP", status.get("estop")),
        ("MANUAL_OVERRIDE", status.get("manual_override")),
    ) if active]
    lines.append(f"  safety flags   : {'+'.join(flags) if flags else 'none'}")
    return lines


def render_dtc(history, limit=5):
    """Render the most recent active DTC records from the history snapshot."""
    records = [item for item in history.get("records", []) if item.get("active")]
    lines = [f"active DTCs      : {len(records)}"]
    for record in records[:limit]:
        lines.append(
            f"  {record.get('code')} {record.get('name')}"
            f" x{record.get('occurrences', 0)}"
            f" action={record.get('safety_action')}"
            f"{' [LATCHED]' if record.get('latched') else ''}")
    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--no-color", action="store_true")
    parser.add_argument("--once", action="store_true",
                        help="print one frame and exit (evidence snapshot)")
    args, ros_args = parser.parse_known_args()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from adas_msgs.msg import McuStatus

    class SafetyHmi(Node):
        def __init__(self):
            super().__init__("safety_hmi")
            self.status = {}
            self.history = {}
            self.create_subscription(McuStatus, "/adas/mcu/status", self.on_status, 10)
            self.create_subscription(String, "/adas/diagnostics/dtc_history",
                                     self.on_history, 10)

        def on_status(self, message):
            self.status = {
                "system_state": message.system_state,
                "active_source": message.active_source,
                "fault_level": message.fault_level,
                "fault_code": message.fault_code,
                "primary_fresh": message.primary_fresh,
                "backup_fresh": message.backup_fresh,
                "aeb_floor_active": message.aeb_floor_active,
                "degraded": message.degraded,
                "manual_override": message.manual_override,
                "estop": message.estop,
                "protocol_version": message.protocol_version,
                "protocol_version_ok": message.protocol_version_ok,
                "test_build": message.test_build,
                "heartbeat_age_s": message.heartbeat_age_s,
                "feedback_age_s": message.feedback_age_s,
                "command_age_s": message.command_age_s,
                "degrade_reason": message.degrade_reason,
            }

        def on_history(self, message):
            try:
                self.history = json.loads(message.data)
            except ValueError:
                self.history = {}

    rclpy.init(args=ros_args)
    node = SafetyHmi()
    colors = not args.no_color and sys.stdout.isatty()
    try:
        while rclpy.ok():
            deadline = time.monotonic() + args.interval
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            frame = render_status(node.status, colors=colors)
            frame += render_dtc(node.history)
            if not args.once:
                sys.stdout.write("\x1b[2J\x1b[H" if colors else "\n")
            print("\n".join(frame), flush=True)
            if args.once:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
