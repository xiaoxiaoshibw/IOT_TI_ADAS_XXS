#!/usr/bin/env python3
"""Deprecated: the v3 HIL session refactor (2026-07) removed the TCP ARM
authorization path. The MCU now self-drives the session to ACTIVE once its CAN
inputs are fresh and safe, and self-recovers on RECOVERY_REQUIRED / FAULT_LOCK.
There is no longer a TCP challenge/commit exchange to invoke, and the
`tcp_arm_*` parameters on the Orin gateway have been removed.

This file is kept only as a marker; it no longer contacts any service and simply
prints a notice so existing scripts invoking it fail informatively.
"""

import sys


def main() -> int:
    print(
        "tcp_arm.py is DEPRECATED: MCU auto-arms the HIL session since the "
        "v3 refactor (2026-07). No TCP ARM service is exposed anymore; nothing "
        "to do. Update your runbook to drop the ARM step.",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())