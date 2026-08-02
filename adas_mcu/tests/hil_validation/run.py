"""HIL 验证脚本入口。

调用方需要：
- 安装 `python-can`、`pyserial`
- 连 USB-CANalyst-II（Linux: `interface="canalystii", channel=1`；Windows 默认 SocketCAN）
- 板端已烧 v0.8.0 镜像（commit `c2cca8c` 或 tag `v0.8.0`）

运行：
    cd tests/hil_validation
    python3 run.py                  # 冒烟 + 跑 T1-T6
    python3 run.py --only T1,T3    # 只跑指定
    python3 run.py --record runs/  # 保存总线原始记录
"""
from __future__ import annotations
import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent

CASES = [
    ("T1", "test_asr_frame_boundary"),
    ("T2", "test_inject_response"),
    ("T3", "test_oled_asr_pixel"),
    ("T4", "test_dgus_present"),
    ("T4b", "test_dgus_present"),
    ("T5", "test_asr_timeout"),
    ("T6", "test_watchdog"),
]


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, str(THIS_DIR / f"{name}.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)                                # type: ignore[union-attr]
    return mod


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--only", help="逗号分隔的用例 ID，如 T1,T3")
    p.add_argument("--record", help="保存总线记录的目录（自动 mkdir）")
    args = p.parse_args()

    # 让 from-import 在 common.py 与 orin_can_reference 之间工作
    sys.path.insert(0, str(THIS_DIR))
    sys.path.insert(0, str(THIS_DIR.parent.parent / "tools"))

    selected = {c.strip() for c in (args.only or ",".join(c[0] for c in CASES)).split(",")}

    results: dict[str, dict] = {}
    for case_id, mod_name in CASES:
        if case_id not in selected:
            continue
        try:
            mod = _load(mod_name)
        except Exception as e:                                  # noqa: BLE001
            print(f"[{case_id}] import {mod_name} FAILED: {e}")
            results[case_id] = {"ok": False, "error": str(e), "duration_s": 0.0}
            continue
        if not hasattr(mod, "run"):
            print(f"[{case_id}] 模块 {mod_name} 缺 run()，跳过")
            continue
        print(f"[{case_id}] running {mod_name}.run() ...")
        t0 = time.time()
        try:
            res = mod.run(record_dir=Path(args.record) if args.record else None)
            ok = bool(res.get("ok"))
        except Exception as e:                                  # noqa: BLE001
            ok, res = False, {"error": str(e)}
        dt = time.time() - t0
        results[case_id] = {"ok": ok, "duration_s": dt, **res}
        print(f"[{case_id}] {'PASS' if ok else 'FAIL'} ({dt:.1f}s)")

    if args.record:
        out_dir = Path(args.record)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "result.json").write_text(json.dumps(results, indent=2, ensure_ascii=False))
        print(f"saved to {out_dir / 'result.json'}")

    bad = [k for k, v in results.items() if not v.get("ok")]
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())

