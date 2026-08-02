"""Pure resource-health policy used by the ROS node and unit tests."""

import glob
import pathlib

LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2


def max_temperature_c():
    """Return the hottest readable thermal zone without failing on bad sysfs nodes."""
    values = []
    for path in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            raw = float(pathlib.Path(path).read_text(encoding="ascii").strip())
            values.append(raw / 1000.0 if raw > 1000.0 else raw)
        except (OSError, TypeError, ValueError):
            continue
    return max(values) if values else None


def evaluate(metrics, thresholds):
    failures = []
    warnings = []

    def low(name, threshold_name, label):
        value = metrics[name]
        if value < thresholds[threshold_name + "_error"]:
            failures.append(f"{label} {value:.1f}% below error threshold")
        elif value < thresholds[threshold_name + "_warn"]:
            warnings.append(f"{label} {value:.1f}% below warning threshold")

    low("memory_available_pct", "memory_available", "memory available")
    low("disk_free_pct", "disk_free", "disk free")

    temperature = metrics.get("max_temperature_c")
    if temperature is not None:
        if temperature >= thresholds["temperature_error_c"]:
            failures.append(f"temperature {temperature:.1f}C above error threshold")
        elif temperature >= thresholds["temperature_warn_c"]:
            warnings.append(f"temperature {temperature:.1f}C above warning threshold")

    load = metrics["normalized_load_1m"]
    if load >= thresholds["normalized_load_error"]:
        failures.append(f"normalized load {load:.2f} above error threshold")
    elif load >= thresholds["normalized_load_warn"]:
        warnings.append(f"normalized load {load:.2f} above warning threshold")

    if not metrics["can_interface_up"]:
        failures.append("CAN interface is not operational")
    if not metrics["log_path_writable"]:
        warnings.append("log path is not writable")

    if failures:
        return LEVEL_ERROR, "; ".join(failures + warnings)
    if warnings:
        return LEVEL_WARN, "; ".join(warnings)
    return LEVEL_OK, "resource health nominal"
