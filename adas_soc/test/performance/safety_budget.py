"""Platform-independent checks for the ADAS HIL timing evidence report."""


MIN_RATE_RATIO = 0.90
MAX_CONTROL_PERIOD_P99_MS = 22.0
MAX_CONTROL_STAGE_P99_MS = 22.0


def _topic_period_p99(report, name):
    return report.get("topics", {}).get(name, {}).get("period", {}).get("p99_ms")


def _stage_p99(report, name):
    return report.get("stages", {}).get(name, {}).get("p99_ms")


def validate_report(report):
    """Return all violated timing gates; an empty list means PASS.

    The monitor reports scheduling evidence only. This check deliberately
    fails absent samples, because an incomplete capture cannot prove timing.
    """
    failures = []
    for name, data in report.get("topics", {}).items():
        expected = data.get("expected_hz", 0.0)
        measured = data.get("measured_hz", 0.0)
        if expected <= 0.0 or measured < expected * MIN_RATE_RATIO:
            failures.append(
                f"{name} rate {measured:.2f}Hz < {MIN_RATE_RATIO * 100:.0f}% of {expected:.1f}Hz")

    for name in ("follower", "gate", "actuation"):
        p99 = _topic_period_p99(report, name)
        if p99 is None:
            failures.append(f"{name} period P99 missing")
        elif p99 > MAX_CONTROL_PERIOD_P99_MS:
            failures.append(
                f"{name} period P99 {p99:.2f}ms > {MAX_CONTROL_PERIOD_P99_MS:.2f}ms")

    for name in ("trajectory_to_follower", "follower_to_gate", "gate_to_actuation"):
        p99 = _stage_p99(report, name)
        if p99 is None:
            failures.append(f"{name} stage P99 missing")
        elif p99 > MAX_CONTROL_STAGE_P99_MS:
            failures.append(
                f"{name} stage P99 {p99:.2f}ms > {MAX_CONTROL_STAGE_P99_MS:.2f}ms")
    return failures
