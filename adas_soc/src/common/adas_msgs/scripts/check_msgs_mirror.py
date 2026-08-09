#!/usr/bin/env python3
"""Fail (exit 1) if the SoC and PC bridge ``adas_msgs`` copies diverge.

Project rule (CLAUDE.md): the two ``adas_msgs`` trees must stay byte-identical
because the ``adas_soc`` ROS package builds from the local copy while CARLA
bridge nodes on the PC build against the mirror copy. Logical content already
matches across the two trees (verified manually for every ``*.msg``/``*.srv``);
this guard catches CRLF/LF drift and any future content divergence.

Exit codes
----------
0  trees agree
1  trees diverge (mismatch emitted to stderr)
2  misconfiguration (missing path or missing tool)
"""

from __future__ import annotations

import argparse
import difflib
import filecmp
import sys
from pathlib import Path

# The two canonical roots. Both live in this repository but in different
# workspaces — the SoC build uses the first, the PC bridge the second.
# Layout (from this script):
#   <repo>/adas_soc/src/common/adas_msgs/scripts/check_msgs_mirror.py
# SoC root is parents[1] (adas_msgs); repo root is parents[5].
DEFAULT_SOC_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BRIDGE_ROOT = (
    Path(__file__).resolve().parents[5]
    / "adas_bridge_pc"
    / "carla_ros2_bridge"
    / "ws"
    / "src"
    / "adas_msgs"
)


def compare_trees(a: Path, b: Path) -> list[str]:
    """Walk both trees and return a list of human-readable diffs.

    Only the *message-bearing* files are compared: ``msg/``, ``srv/``, and
    ``package.xml``. ``CMakeLists.txt`` and any helper ``scripts/`` are build
    glue that legitimately differs between the SoC and bridge workspaces
    (different rosidl deps, SoC-only test hooks, etc.). Comparing them would
    surface false positives on every workspace-local change.
    """
    diffs: list[str] = []
    if not a.is_dir():
        return [f"SoC adas_msgs root missing: {a}"]
    if not b.is_dir():
        return [f"Bridge adas_msgs root missing: {b}"]

    # Files that participate in the message-interface contract. Both copies
    # must agree on these byte-for-byte.
    def is_contract(rel: Path) -> bool:
        parts = rel.parts
        if not parts:
            return False
        if parts[0] in {"msg", "srv"}:
            return True
        if rel == Path("package.xml"):
            return True
        return False

    def collect(root: Path) -> set[Path]:
        return {p.relative_to(root) for p in root.rglob("*")
                if p.is_file() and is_contract(p.relative_to(root))}

    a_files = collect(a)
    b_files = collect(b)
    only_a = sorted(a_files - b_files)
    only_b = sorted(b_files - a_files)
    for rel in only_a:
        diffs.append(f"only in SoC:     {rel}")
    for rel in only_b:
        diffs.append(f"only in bridge: {rel}")

    common = sorted(a_files & b_files)
    for rel in common:
        pa, pb = a / rel, b / rel
        if pa.read_bytes() == pb.read_bytes():
            continue
        # Byte mismatch. Try a unified diff so the message is actionable.
        diff_text = "".join(
            difflib.unified_diff(
                pa.read_text(encoding="utf-8", errors="replace").splitlines(keepends=True),
                pb.read_text(encoding="utf-8", errors="replace").splitlines(keepends=True),
                fromfile=str(pa),
                tofile=str(pb),
                n=2,
            )
        ).strip()
        head = diff_text.splitlines()[0] if diff_text else "byte mismatch"
        diffs.append(f"differs:        {rel}  ({head})")
    return diffs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--soc", type=Path, default=DEFAULT_SOC_ROOT,
                        help=f"SoC adas_msgs root (default: {DEFAULT_SOC_ROOT})")
    parser.add_argument("--bridge", type=Path, default=DEFAULT_BRIDGE_ROOT,
                        help=f"PC bridge adas_msgs root (default: {DEFAULT_BRIDGE_ROOT})")
    args = parser.parse_args()

    if not args.soc.exists():
        print(f"ERROR: SoC root does not exist: {args.soc}", file=sys.stderr)
        return 2
    if not args.bridge.exists():
        print(f"ERROR: bridge root does not exist: {args.bridge}", file=sys.stderr)
        return 2

    diffs = compare_trees(args.soc, args.bridge)
    if not diffs:
        print(f"OK: {args.soc} and {args.bridge} are byte-identical.",
              file=sys.stderr)
        return 0
    print("adas_msgs copies diverge:", file=sys.stderr)
    for line in diffs:
        print(f"  {line}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
