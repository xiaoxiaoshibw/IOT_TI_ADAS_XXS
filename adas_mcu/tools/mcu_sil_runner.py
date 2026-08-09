#!/usr/bin/env python3
"""Build and run the MCU safety/control core against a Linux vcan interface.

The host binary links the production C modules (CAN decoder, safety arbiter,
HIL session gate and control mapper).  Only the C2000 driverlib CAN calls are
replaced by tools/host/driverlib.c, which maps those calls to SocketCAN.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile


def compile_host_binary(mcu_root: Path, output: Path) -> None:
    include_dir = mcu_root / "include"
    host_dir = mcu_root / "tools" / "host"
    sources = [
        mcu_root / "tools" / "mcu_sil_main.c",
        host_dir / "driverlib.c",
        mcu_root / "src" / "crc8.c",
        mcu_root / "src" / "can_comm.c",
        mcu_root / "src" / "control.c",
        mcu_root / "src" / "safety.c",
        mcu_root / "src" / "hil_session.c",
        mcu_root / "src" / "self_test.c",
    ]
    command = [
        "gcc",
        "-std=c99",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-D_DEFAULT_SOURCE",
        "-D_POSIX_C_SOURCE=200809L",
        "-DADAS_HOST_TEST=1",
        "-DADAS_TEST_BUILD=1",
        "-I",
        str(host_dir),
        "-I",
        str(include_dir),
        *(str(source) for source in sources),
        "-o",
        str(output),
    ]
    subprocess.run(command, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="vcan0")
    parser.add_argument(
        "--duration",
        type=int,
        default=0,
        help="run for this many seconds; 0 means until launch shutdown",
    )
    parser.add_argument("--keep-build", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration < 0:
        raise SystemExit("--duration must be non-negative")
    mcu_root = Path(__file__).resolve().parents[1]
    temporary = None
    if args.keep_build:
        build_dir = mcu_root / ".sil_build"
        build_dir.mkdir(exist_ok=True)
    else:
        temporary = tempfile.TemporaryDirectory(prefix="adas_mcu_sil_")
        build_dir = Path(temporary.name)
    binary = build_dir / "mcu_sil_host"

    try:
        print("=== build MCU SIL host binary ===", flush=True)
        compile_host_binary(mcu_root, binary)
        environment = os.environ.copy()
        environment["ADAS_MCU_CAN_INTERFACE"] = args.interface
        command = [str(binary)]
        if args.duration:
            command.extend(["--duration", str(args.duration)])
        child = subprocess.Popen(command, env=environment)

        def stop_child(signum: int, _frame: object) -> None:
            if child.poll() is None:
                child.send_signal(signum)

        signal.signal(signal.SIGINT, stop_child)
        signal.signal(signal.SIGTERM, stop_child)
        return child.wait()
    finally:
        if temporary is not None:
            temporary.cleanup()


if __name__ == "__main__":
    sys.exit(main())
