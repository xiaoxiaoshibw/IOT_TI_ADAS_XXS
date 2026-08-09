"""Pure resource-health policy used by the ROS node and unit tests."""

import glob
import os
import pathlib

LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2
GPU_TEMPERATURE_PATHS = (
    "/sys/devices/gpu.0/thermal/temp",
    "/sys/devices/platform/gpu.0/thermal/temp",
)


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


def gpu_temperature_c(paths=GPU_TEMPERATURE_PATHS):
    """Return the hottest readable Jetson GPU thermal sensor, if present."""
    values = []
    for path in paths:
        try:
            raw = float(pathlib.Path(path).read_text(encoding="ascii").strip())
            values.append(raw / 1000.0 if raw > 1000.0 else raw)
        except (OSError, TypeError, ValueError):
            continue
    return max(values) if values else None


def read_meminfo(path="/proc/meminfo"):
    values = {}
    with open(path, encoding="ascii") as stream:
        for line in stream:
            name, value = line.split(":", 1)
            values[name] = int(value.strip().split()[0])
    return values


def swap_used_pct(path="/proc/meminfo"):
    """Return used swap percentage; a swap-less system is reported as 0%."""
    values = read_meminfo(path)
    total = values.get("SwapTotal", 0)
    if total <= 0:
        return 0.0
    free = values.get("SwapFree", 0)
    return max(0.0, min(100.0, 100.0 * (total - free) / total))


def can_operstate_is_healthy(state):
    """SocketCAN must explicitly report up; unknown is not a safe state."""
    return state == "up"


def process_snapshot(process_names, proc_root="/proc", clock_ticks=None, page_size=None):
    """Read CPU time and RSS for configured processes from procfs.

    The returned CPU value is cumulative process time.  The node turns two
    snapshots into a percentage, keeping this function deterministic and
    straightforward to unit test on non-Linux hosts.
    """
    wanted = set(process_names)
    if not wanted:
        return {}
    root = pathlib.Path(proc_root)
    clock_ticks = clock_ticks or os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    page_size = page_size or os.sysconf(os.sysconf_names["SC_PAGE_SIZE"])
    result = {}
    try:
        entries = list(root.iterdir())
    except OSError:
        return result
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            name = (entry / "comm").read_text(encoding="ascii").strip()
            if name not in wanted:
                continue
            fields = (entry / "stat").read_text(encoding="ascii").split()
            utime = int(fields[13])
            stime = int(fields[14])
            rss_pages = int(fields[23])
            result[name] = {
                "pid": int(entry.name),
                "cpu_time_s": (utime + stime) / float(clock_ticks),
                "rss_mb": rss_pages * page_size / (1024.0 * 1024.0),
            }
        except (OSError, IndexError, TypeError, ValueError, KeyError):
            continue
    return result


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

    gpu_temperature = metrics.get("gpu_temperature_c")
    if gpu_temperature is not None:
        if gpu_temperature >= thresholds.get("gpu_temperature_error_c",
                                             thresholds["temperature_error_c"]):
            failures.append(f"GPU temperature {gpu_temperature:.1f}C above error threshold")
        elif gpu_temperature >= thresholds.get("gpu_temperature_warn_c",
                                               thresholds["temperature_warn_c"]):
            warnings.append(f"GPU temperature {gpu_temperature:.1f}C above warning threshold")

    swap = metrics.get("swap_used_pct")
    if swap is not None:
        if swap >= thresholds.get("swap_used_error", 50.0):
            failures.append(f"swap used {swap:.1f}% above error threshold")
        elif swap >= thresholds.get("swap_used_warn", 20.0):
            warnings.append(f"swap used {swap:.1f}% above warning threshold")

    load = metrics["normalized_load_1m"]
    if load >= thresholds["normalized_load_error"]:
        failures.append(f"normalized load {load:.2f} above error threshold")
    elif load >= thresholds["normalized_load_warn"]:
        warnings.append(f"normalized load {load:.2f} above warning threshold")

    if not metrics["can_interface_up"]:
        failures.append("CAN interface is not operational")
    if not metrics["log_path_writable"]:
        warnings.append("log path is not writable")

    processes = metrics.get("critical_processes", {})
    for name in metrics.get("required_processes", []):
        if name not in processes:
            failures.append(f"critical process {name} is missing")
    process_values = list(processes.values())
    if process_values:
        max_cpu = max(item.get("cpu_pct", 0.0) for item in process_values)
        max_rss = max(item.get("rss_mb", 0.0) for item in process_values)
        if max_cpu >= thresholds.get("process_cpu_error_pct", 95.0):
            failures.append(f"critical process CPU {max_cpu:.1f}% above error threshold")
        elif max_cpu >= thresholds.get("process_cpu_warn_pct", 80.0):
            warnings.append(f"critical process CPU {max_cpu:.1f}% above warning threshold")
        if max_rss >= thresholds.get("process_rss_error_mb", 2048.0):
            failures.append(f"critical process RSS {max_rss:.1f}MB above error threshold")
        elif max_rss >= thresholds.get("process_rss_warn_mb", 1024.0):
            warnings.append(f"critical process RSS {max_rss:.1f}MB above warning threshold")

    if failures:
        return LEVEL_ERROR, "; ".join(failures + warnings)
    if warnings:
        return LEVEL_WARN, "; ".join(warnings)
    return LEVEL_OK, "resource health nominal"
