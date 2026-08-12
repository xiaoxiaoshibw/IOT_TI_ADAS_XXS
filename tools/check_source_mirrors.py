#!/usr/bin/env python3
"""Fail when deployable SoC sources drift from their Orin mirror."""

from __future__ import annotations

import argparse
import filecmp
from pathlib import Path


MIRRORED_PATHS = (
    "common/adas_msgs/CMakeLists.txt",
    "common/adas_msgs/package.xml",
    "common/adas_msgs/msg",
    "common/adas_msgs/srv",
    "planning/adas_global_planner/CMakeLists.txt",
    "planning/adas_global_planner/include",
    "planning/adas_global_planner/src",
    "planning/adas_global_planner/test",
    "simulation/adas_sim_vehicle/CMakeLists.txt",
    "simulation/adas_sim_vehicle/package.xml",
    "simulation/adas_sim_vehicle/src",
    "launch/adas_launch/launch/carla.launch.py",
    "launch/adas_launch/launch/local_three_machine.launch.py",
    "launch/adas_launch/launch/local_three_machine_carla.launch.py",
    "launch/adas_launch/launch/sil_launch_common.py",
)


def differing_files(authoritative: Path, mirror: Path) -> list[str]:
    differences: list[str] = []
    for relative in MIRRORED_PATHS:
        left = authoritative / relative
        right = mirror / relative
        if left.is_dir():
            comparison = filecmp.dircmp(left, right)
            if comparison.left_only or comparison.right_only or comparison.diff_files:
                differences.append(relative)
                continue
            for candidate in left.rglob("*"):
                if candidate.is_file():
                    peer = right / candidate.relative_to(left)
                    if not peer.is_file() or candidate.read_bytes() != peer.read_bytes():
                        differences.append(str(Path(relative) / candidate.relative_to(left)))
        elif not right.is_file() or left.read_bytes() != right.read_bytes():
            differences.append(relative)
    return sorted(set(differences))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path,
                        default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    root = args.repo_root.resolve()
    differences = differing_files(
        root / "adas_soc/src",
        root / "adas_bridge_pc/carla_ros2_bridge/Orin同步/src")
    authoritative_msgs = root / "adas_soc/src/common/adas_msgs"
    pc_msgs = root / "adas_bridge_pc/carla_ros2_bridge/ws/src/adas_msgs"
    for relative in ("package.xml", "msg", "srv"):
        left = authoritative_msgs / relative
        right = pc_msgs / relative
        if left.is_dir():
            left_files = {p.relative_to(left): p.read_bytes()
                          for p in left.rglob("*") if p.is_file()}
            right_files = {p.relative_to(right): p.read_bytes()
                           for p in right.rglob("*") if p.is_file()}
            if left_files != right_files:
                differences.append(f"PC adas_msgs/{relative}")
        elif not right.is_file() or left.read_bytes() != right.read_bytes():
            differences.append(f"PC adas_msgs/{relative}")
    if differences:
        print("FAIL: SoC/Orin source drift:")
        for path in differences:
            print(f"  {path}")
        return 1
    print(f"PASS: {len(MIRRORED_PATHS)} SoC/Orin contracts and all three "
          "adas_msgs copies are identical")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
