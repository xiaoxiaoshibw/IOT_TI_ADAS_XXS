#!/usr/bin/env python3
"""Temporary runtime wrapper for Jetson thermal sysfs read failures."""

import glob
import pathlib

from adas_resource_monitor import node


def safe_max_temperature_c():
    values = []
    for path in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            raw = float(pathlib.Path(path).read_text(encoding="ascii").strip())
            values.append(raw / 1000.0 if raw > 1000.0 else raw)
        except (OSError, TypeError, ValueError):
            continue
    return max(values) if values else None


node.max_temperature_c = safe_max_temperature_c

original_evaluate = node.evaluate


def compatible_evaluate(metrics, thresholds):
    level, message = original_evaluate(metrics, thresholds)
    return bytes([level]), message


node.evaluate = compatible_evaluate
node.main()
