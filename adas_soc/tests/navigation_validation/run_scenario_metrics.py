#!/usr/bin/env python3
"""Per-scenario metrics entry point for the SIL/HIL validation plan.

Typical usage (Commit 1d):

    # 1. start SIL fallback in one terminal (or in --check mode):
    ./scripts/run_sil_fallback.sh --scenario acc --duration 90 &

    # 2. record a synchronized CSV during the run:
    python3 adas_soc/tests/navigation_validation/data_logger.py \\
        --output logs/sil/acc_test.csv --rate 20 --duration 90

    # 3. compute metrics and assert the user's acceptance gates:
    python3 adas_soc/tests/navigation_validation/run_scenario_metrics.py \\
        --csv logs/sil/acc_test.csv \\
        --scenario-name acc_test \\
        --output-json logs/sil/acc_test.json \\
        --road-class straight

Acceptance gates (per Commit 1 baseline):

  straight:   P95 |lat| < 0.10 m
  bend:       max |lat| < 0.50 m, P95 < 0.25 m
              lane heading jumps < 0.78 rad (45 deg)
  steering:   P95 |rate| < 0.20 rad/s, max < 0.40 rad/s
  comfort:    jerk P95 < 2.0 m/s^3 (non-AEB)
  safety/AEB: counted separately; not gated here

The `--road-class` flag selects which subset of gates to enforce:

  auto       choose from max(|lateral|) — straight if <=0.10 m,
              bend otherwise. Useful when the scenario mixes both.
  straight   straight-only gates (P95 |lat|, jerk, steer rate)
  bend       bend gates (max |lat|, P95 |lat|)
  any        enforce no scenario-specific gates; just produce JSON.

Exit codes
----------
0   all gates for the road-class satisfied
1   one or more gates failed
2   script / input error (missing csv, empty rows)
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "adas_soc" / "tests" / "navigation_validation"))
from validation_common import navigation_metrics  # noqa: E402


# --- Acceptance gates (Commit 1 baseline) ---

GATES = {
    "straight": [
        ("lane_lateral_offset_abs_p95_m", "<", 0.10,
         "P95 |lateral offset| < 0.10 m on straight"),
    ],
    "bend": [
        ("lane_lateral_offset_abs_max_m", "<", 0.50,
         "max |lateral offset| < 0.50 m on bend"),
        ("lane_lateral_offset_abs_p95_m", "<", 0.25,
         "P95 |lateral offset| < 0.25 m on bend"),
        ("lane_heading_error_abs_max_rad", "<", 0.785398,
         "max |heading error| < 45 deg (lane heading jumps)"),
    ],
    "any": [
        ("steering_rate_p95_rad_s", "<", 0.20,
         "P95 |steer rate| < 0.20 rad/s"),
        ("steering_rate_max_abs_rad_s", "<", 0.40,
         "max |steer rate| < 0.40 rad/s"),
        ("jerk_p95_abs_mps3", "<", 2.0,
         "P95 |jerk| < 2.0 m/s^3 (comfort)"),
    ],
}


def _read_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _auto_road_class(metrics: dict) -> str:
    """Pick straight vs bend from the captured lateral offset max."""
    max_off = metrics.get("lane_lateral_offset_abs_max_m")
    if max_off is None:
        # No lane data — fall back to "any" so we still produce JSON.
        return "any"
    return "straight" if max_off <= 0.10 else "bend"


def _evaluate(metrics: dict, road_class: str) -> list[dict]:
    """Return a list of gate-result dicts. Missing keys evaluate as 'skipped'.

    The "any" gates (steering rate, jerk) apply universally. The
    straight/bend gates apply only when that road-class was selected.
    Straight is stricter than bend and should not be evaluated alongside
    bend (otherwise the displayed gate list becomes misleading).
    """
    results: list[dict] = []

    # Universal gates always apply.
    for key, op, threshold, descr in GATES["any"]:
        value = metrics.get(key)
        passed = value is not None and value < threshold
        results.append({
            "gate": key,
            "scope": "any",
            "operator": op,
            "threshold": threshold,
            "value": value,
            "passed": passed,
            "skipped": value is None,
            "description": descr,
        })

    # Scenario-specific gates. "any" road-class skips these.
    if road_class in ("straight", "bend"):
        for key, op, threshold, descr in GATES[road_class]:
            value = metrics.get(key)
            passed = value is not None and value < threshold
            results.append({
                "gate": key,
                "scope": road_class,
                "operator": op,
                "threshold": threshold,
                "value": value,
                "passed": passed,
                "skipped": value is None,
                "description": descr,
            })
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path,
                        help="Phase-3 data_logger CSV to analyze")
    parser.add_argument("--output-json", type=Path, default=None,
                        help="Where to write the metrics JSON (default: <csv>.metrics.json)")
    parser.add_argument("--scenario-name", default="",
                        help="Optional scenario name written into the JSON metadata")
    parser.add_argument("--road-class", choices=("auto", "straight", "bend", "any"),
                        default="auto", help="Which subset of gates to enforce")
    parser.add_argument("--tag", default="",
                        help="Optional label written to the JSON (used by Commit 7 "
                             "A/B to identify the controller, e.g. 'pp' or 'lqr')")
    args = parser.parse_args()

    if not args.csv.exists():
        print(f"ERROR: csv does not exist: {args.csv}", file=sys.stderr)
        return 2

    rows = _read_rows(args.csv)
    if not rows:
        print(f"ERROR: csv is empty: {args.csv}", file=sys.stderr)
        return 2

    metrics = navigation_metrics(rows)
    metrics["source_csv"] = str(args.csv)
    metrics["scenario_name"] = args.scenario_name
    metrics["tag"] = args.tag

    road_class = args.road_class
    if road_class == "auto":
        road_class = _auto_road_class(metrics)
    metrics["road_class_applied"] = road_class

    gates = _evaluate(metrics, road_class)
    metrics["gates"] = gates

    out_path = args.output_json or args.csv.with_suffix(".metrics.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")

    failed = [g for g in gates if not g["passed"] and not g["skipped"]]
    skipped = [g for g in gates if g["skipped"]]
    print(f"scenario: {args.scenario_name or args.csv.stem}")
    print(f"road_class: {road_class}")
    print(f"samples: {metrics.get('sample_count')}")
    for g in gates:
        marker = ("PASS" if g["passed"] else
                  ("SKIP" if g["skipped"] else "FAIL"))
        value_str = "n/a" if g["value"] is None else f"{g['value']:.4f}"
        print(f"  [{marker}] {g['gate']:<42s} {value_str:<10s} "
              f"< {g['threshold']:.4f}  -- {g['description']}")
    if failed:
        print(f"FAILED: {len(failed)} gate(s) below threshold", file=sys.stderr)
        return 1
    if skipped and road_class != "any":
        print(f"NOTE: {len(skipped)} gate(s) skipped (missing input data)")
    print(f"OK: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
