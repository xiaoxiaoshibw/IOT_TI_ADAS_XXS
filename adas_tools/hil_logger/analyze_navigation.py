#!/usr/bin/env python3
"""Compute Phase 3 metrics strictly from captured CSV evidence."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ADAS0.0.2/SoC/tests/navigation_validation"))
from validation_common import atomic_json, navigation_metrics  # noqa: E402


def read_rows(path: Path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def can_metrics(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {
            "can_metrics_source": None,
            "can_frame_rate_hz": None,
            "can_error_delta": None,
            "can_drop_delta": None,
            "can_bus_error_delta": None,
        }
    rows = read_rows(path)
    if not rows:
        raise ValueError(f"empty CAN metrics: {path}")
    first, last = rows[0], rows[-1]
    elapsed = float(last["time"]) - float(first["time"])
    rx_delta = int(last["rx_count"]) - int(first["rx_count"])
    return {
        "can_metrics_source": str(path),
        "can_frame_rate_hz": rx_delta / elapsed if elapsed > 0.0 else None,
        "can_error_delta": int(last["error"]) - int(first["error"]),
        "can_drop_delta": int(last["drop"]) - int(first["drop"]),
        "can_bus_error_delta": int(last["bus_error"]) - int(first["bus_error"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--can-metrics", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--markdown", type=Path)
    args = parser.parse_args()
    rows = read_rows(args.csv)
    if not rows:
        parser.error("navigation CSV is empty")
    metrics = {
        "schema": 1,
        "source_csv": str(args.csv),
        "real_measurements_only": True,
        **navigation_metrics(rows),
        **can_metrics(args.can_metrics),
    }
    crc = [int(row["crc_error"]) for row in rows if row.get("crc_error") not in (None, "")]
    metrics["mcu_crc_error_delta"] = crc[-1] - crc[0] if crc else None
    sources = [int(row["mcu_active_source"]) for row in rows if row.get("mcu_active_source") not in (None, "")]
    metrics["mcu_source_changes"] = sum(a != b for a, b in zip(sources, sources[1:])) if sources else None
    atomic_json(args.output, metrics)
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        def shown(key, unit=""):
            value = metrics.get(key)
            return "TODO（无真实采集数据）" if value is None else f"{value:.4f}{unit}" if isinstance(value, float) else str(value)
        args.markdown.write_text(
            "# Phase 3 实验统计（自动生成）\n\n"
            f"- 数据源：`{args.csv}`\n"
            f"- 样本数：{shown('sample_count')}\n"
            f"- 横向误差 RMS：{shown('lateral_error_rms_m', ' m')}\n"
            f"- 最大横向误差：{shown('lateral_error_max_abs_m', ' m')}\n"
            f"- 速度误差 RMS：{shown('speed_error_rms_mps', ' m/s')}\n"
            f"- 行驶距离：{shown('travel_distance_m', ' m')}\n"
            f"- CAN drop 增量：{shown('can_drop_delta')}\n"
            f"- MCU CRC 错误增量：{shown('mcu_crc_error_delta')}\n\n"
            "> 本页仅由 CSV 计算；缺失项保留 TODO，不作估算。\n",
            encoding="utf-8",
        )
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

