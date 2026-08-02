#!/usr/bin/env python3
"""
ADAS HIL 测量套件 — 自动化执行 + 数据采集 + 报告生成
=====================================================
所有测量方法严格遵循《实验数据唯一口径表.md》的定义。

用法:
  # 运行全部测量
  python3 run_hil_measurements.py --all

  # 运行单项测量
  python3 run_hil_measurements.py --can-period
  python3 run_hil_measurements.py --cpu-load
  python3 run_hil_measurements.py --fault-matrix
  python3 run_hil_measurements.py --cold-boot

  # 仅分析已有证据
  python3 run_hil_measurements.py --analyze-only --evidence-dir ../../../../文档/ADAS报告修订包/evidence/
"""

import argparse
import csv
import json
import os
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

# ============================================================================
# 统计工具
# ============================================================================

def compute_stats(values):
    """按《实验数据唯一口径表》计算统计量"""
    n = len(values)
    if n == 0:
        return {}
    sorted_v = sorted(values)
    return {
        "n": n,
        "mean": sum(values) / n,
        "min": sorted_v[0],
        "max": sorted_v[-1],
        "p50": sorted_v[n // 2],
        "p95": sorted_v[int(n * 0.95)],
        "p99": sorted_v[int(n * 0.99)],
        "sigma": (sum((x - sum(values)/n)**2 for x in values) / n)**0.5,
    }


def print_stats(name, stats, unit=""):
    """打印统计摘要"""
    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"{'='*60}")
    print(f"  样本数:  {stats['n']}")
    print(f"  均值:    {stats['mean']:.3f}{unit}")
    print(f"  标准差:  {stats['sigma']:.3f}{unit}")
    print(f"  最小值:  {stats['min']:.3f}{unit}")
    print(f"  P50:     {stats['p50']:.3f}{unit}")
    print(f"  P95:     {stats['p95']:.3f}{unit}")
    print(f"  P99:     {stats['p99']:.3f}{unit}")
    print(f"  最大值:  {stats['max']:.3f}{unit}")


# ============================================================================
# 证据分析（无需硬件）
# ============================================================================

def analyze_frame_period(csv_path):
    """分析 CAN 帧周期（口径 3.1 约定: 用接收时间戳差）"""
    print(f"\n📊 分析帧周期数据: {csv_path}")

    periods = defaultdict(list)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            can_id = row['can_id'].strip()
            delta = float(row['delta_ms'])
            periods[can_id].append(delta)

    for can_id, vals in sorted(periods.items()):
        stats = compute_stats(vals)
        print_stats(f"CAN {can_id} 帧周期", stats, " ms")

    return periods


def analyze_cpu_load(csv_path):
    """分析 MCU CPU 负载"""
    print(f"\n📊 分析 CPU 负载数据: {csv_path}")

    loads = []
    states = defaultdict(int)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            load = int(row['loop_load_pct'])
            loads.append(load)
            state = int(row['state'])
            states[state] += 1

    stats = compute_stats(loads)
    print_stats("MCU 自报 CPU 负载 (loop_load_pct)", stats, "%")
    print(f"\n  状态分布: {dict(states)}")

    return loads


def analyze_cold_boot(csv_path):
    """分析冷启动会话推进"""
    print(f"\n📊 分析冷启动会话数据: {csv_path}")

    phases = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            phases.append({
                't': float(row['t_s']),
                'phase': int(row['phase']),
                'session_id': int(row['session_id']),
            })

    # 找出状态跳变时刻
    transitions = []
    prev_phase = None
    for p in phases:
        if prev_phase is not None and p['phase'] != prev_phase:
            transitions.append({
                'from': prev_phase,
                'to': p['phase'],
                't': p['t'],
            })
        prev_phase = p['phase']

    print(f"  总记录数: {len(phases)}")
    print(f"  状态跳变: {len(transitions)} 次")
    for t in transitions:
        print(f"    {t['from']} → {t['to']} @ {t['t']:.3f}s")

    return phases, transitions


def analyze_bench_rtt(csv_path):
    """分析 Bench RTT 数据"""
    print(f"\n📊 分析 Bench RTT 数据: {csv_path}")
    return _analyze_csv_load_groups(csv_path)


def _analyze_csv_load_groups(csv_path):
    """通用: 按 load 分组分析"""
    groups = defaultdict(list)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            load = int(row.get('load', 0))
            rtt = float(row.get('rtt_us', row.get('delta_ms', 0)))
            groups[load].append(rtt)

    for load in sorted(groups):
        stats = compute_stats(groups[load])
        unit = " µs" if max(groups[load]) > 100 else " ms"
        print_stats(f"  Load={load}", stats, unit)

    return groups


# ============================================================================
# 硬件测量（需要 CAN 硬件连接）
# ============================================================================

class CanProbe:
    """CAN 探测基类"""

    def __init__(self, interface="can1", bitrate=500000):
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self._can_available = False

    def check_hardware(self):
        """检查 CAN 硬件是否可用"""
        try:
            import can
            available = []
            # 尝试 SocketCAN
            try:
                bus = can.Bus(interface="socketcan", channel=self.interface,
                              bitrate=self.bitrate)
                bus.send(can.Message(arbitration_id=0x7FF, data=[0], is_extended_id=False))
                bus.shutdown()
                available.append("socketcan")
            except Exception:
                pass
            # 尝试 canalystii
            try:
                bus = can.Bus(interface="canalystii", channel=1, bitrate=self.bitrate)
                bus.shutdown()
                available.append("canalystii")
            except Exception:
                pass
            self._can_available = len(available) > 0
            return available
        except ImportError:
            print("⚠  python-can 未安装，无法进行硬件测量")
            self._can_available = False
            return []

    def measure_frame_period(self, can_id=0x201, duration_s=30):
        """测量指定 CAN ID 的帧到达周期（口径 3.1）"""
        import can
        print(f"\n🔬 测量 {hex(can_id)} 帧周期 ({duration_s}s)...")
        try:
            bus = can.Bus(interface="socketcan", channel=self.interface,
                          bitrate=self.bitrate)
        except Exception:
            bus = can.Bus(interface="canalystii", channel=1,
                          bitrate=self.bitrate)

        start = time.time()
        last_t = None
        records = []
        msg_count = 0

        while time.time() - start < duration_s:
            msg = bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == can_id:
                now = time.time()
                if last_t is not None:
                    delta_ms = (now - last_t) * 1000
                    records.append({
                        't_s': now - start,
                        'can_id': hex(can_id),
                        'delta_ms': round(delta_ms, 4),
                    })
                last_t = now
                msg_count += 1

        bus.shutdown()
        print(f"  收到 {msg_count} 帧，记录 {len(records)} 个周期")

        if records:
            vals = [r['delta_ms'] for r in records]
            stats = compute_stats(vals)
            print_stats(f"CAN {hex(can_id)} 帧周期实测", stats, " ms")

        return records


# ============================================================================
# 报告生成
# ============================================================================

def generate_evidence_report(evidence_dir, output_dir="."):
    """从 evidence 目录生成综合分析报告"""
    evidence_dir = Path(evidence_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    report = {
        "report_time": datetime.now().isoformat(),
        "evidence_dir": str(evidence_dir),
        "measurements": {},
        "gaps": [],
    }

    # 分析每份证据
    analysis_map = {
        "frame_period": ("evidence_frame_period_raw.csv", analyze_frame_period),
        "cpu_load": ("evidence_cpu_load_raw.csv", analyze_cpu_load),
        "cold_boot": ("evidence_hil_session_coldboot_raw.csv", analyze_cold_boot),
        "hil_session": ("evidence_hil_session_raw.csv", analyze_cold_boot),
        "long_watch": ("evidence_hil_session_longwatch_raw.csv", analyze_cold_boot),
        "bench_rtt": None,  # 需要跑 bench.py
    }

    for name, entry in analysis_map.items():
        if entry is None:
            continue
        filename, analyzer = entry
        fpath = evidence_dir / filename
        if fpath.exists():
            print(f"\n{'#'*60}")
            print(f"# {name}")
            print(f"{'#'*60}")
            try:
                result = analyzer(str(fpath))
                report["measurements"][name] = {
                    "file": str(fpath),
                    "status": "analyzed",
                    "result_type": type(result).__name__,
                }
            except Exception as e:
                print(f"  ⚠ 分析失败: {e}")
                report["measurements"][name] = {
                    "file": str(fpath),
                    "status": "failed",
                    "error": str(e),
                }
        else:
            print(f"  ⚠ {filename} 未找到")
            report["gaps"].append(filename)

    # 记录缺失项
    missing = [
        "MCU ISR 执行时间（需 GPIO 翻转 + 逻辑分析仪）",
        "CCS Profiler 独立 CPU 占用率",
        "CAN RTT（Orin↔MCU 链路）",
        "主源超时端到端接管时延",
        "Seq 停滞端到端接管时延",
        "双源失效制动响应",
        "ESP32 vs F280025C 对比基准",
        "MCU→PC→CARLA 完整闭环录像",
    ]
    report["gaps"].extend(missing)

    # 保存报告
    report_path = output_dir / "evidence_report.json"
    with open(report_path, 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"\n📄 报告已保存: {report_path}")

    # 打印缺口
    print(f"\n{'!'*60}")
    print(f"  待完成项 ({len(report['gaps'])} 项)")
    print(f"{'!'*60}")
    for i, gap in enumerate(report['gaps'], 1):
        print(f"  {i}. {gap}")

    return report


# ============================================================================
# CLI 入口
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="ADAS HIL 测量套件")
    parser.add_argument("--all", action="store_true", help="运行全部测量")
    parser.add_argument("--can-period", action="store_true", help="CAN 帧周期测量")
    parser.add_argument("--cpu-load", action="store_true", help="CPU 负载测量")
    parser.add_argument("--fault-matrix", action="store_true", help="故障注入矩阵")
    parser.add_argument("--cold-boot", action="store_true", help="冷启动测量")
    parser.add_argument("--interface", default="can1", help="SocketCAN 接口")
    parser.add_argument("--analyze-only", action="store_true",
                        help="仅分析已有证据（无需硬件）")
    parser.add_argument("--evidence-dir",
                        default="../../../../文档/ADAS报告修订包/evidence/",
                        help="证据目录路径")
    parser.add_argument("--output", default=".", help="输出目录")

    args = parser.parse_args()

    # 默认：仅分析
    if not any([args.all, args.can_period, args.cpu_load,
                args.fault_matrix, args.cold_boot]):
        args.analyze_only = True

    # 分析模式
    if args.analyze_only or args.all:
        print("=" * 60)
        print("  ADAS HIL 证据分析")
        print("=" * 60)

        ev_dir = os.path.abspath(args.evidence_dir)
        if os.path.exists(ev_dir):
            generate_evidence_report(ev_dir, args.output)
        else:
            # 尝试相对路径
            alt_dir = os.path.join(os.path.dirname(__file__), args.evidence_dir)
            if os.path.exists(alt_dir):
                generate_evidence_report(alt_dir, args.output)
            else:
                print(f"⚠ 证据目录未找到: {ev_dir}")
                print(f"   也尝试了: {alt_dir}")
                print("  使用 --evidence-dir 指定正确路径")

    # 硬件模式
    if args.all or args.can_period or args.cpu_load or args.fault_matrix or args.cold_boot:
        probe = CanProbe(interface=args.interface)
        avail = probe.check_hardware()
        if not avail:
            print("\n⚠  CAN 硬件未连接。请在 Jetson Orin 上运行。")
            print("   可用接口: socketcan(can1), canalystii(ch1)")
            return

        print(f"\n✅ CAN 硬件就绪: {avail}")

        if args.all or args.can_period:
            probe.measure_frame_period(0x201, duration_s=30)
            probe.measure_frame_period(0x202, duration_s=30)

        if args.all or args.cold_boot:
            print("\n🔬 冷启动测量需要物理复位 MCU。")
            print("   请按 S1 复位键，脚本会自动检测 0x206 变化...")
            # 冷启动测量逻辑
            import can
            try:
                bus = can.Bus(interface="socketcan", channel=args.interface,
                              bitrate=500000)
            except Exception:
                bus = can.Bus(interface="canalystii", channel=1,
                              bitrate=500000)

            print("   等待 0x206 帧 (timeout=30s)...")
            start = time.time()
            records = []
            while time.time() - start < 30:
                msg = bus.recv(timeout=0.5)
                if msg and msg.arbitration_id == 0x206:
                    records.append({
                        't_s': round(time.time() - start, 4),
                        'phase': msg.data[0],
                        'session_id': msg.data[1] | (msg.data[2] << 8),
                    })
                    phase_names = {0: "BOOT", 1: "ACCEPT", 2: "READY",
                                   3: "ARMED", 4: "ACTIVE", 5: "RECOVERY"}
                    pname = phase_names.get(msg.data[0], f"UNKNOWN({msg.data[0]})")
                    print(f"      {msg.data[0]}:{pname}  "
                          f"session=0x{records[-1]['session_id']:04X}  "
                          f"t={records[-1]['t_s']:.3f}s")

            bus.shutdown()

            # 保存
            csv_path = os.path.join(args.output, "hil_cold_boot_raw.csv")
            if records:
                with open(csv_path, 'w', newline='') as f:
                    w = csv.DictWriter(f, fieldnames=['t_s', 'phase', 'session_id'])
                    w.writeheader()
                    w.writerows(records)
                print(f"\n  已保存: {csv_path}")

        if args.all or args.cpu_load:
            print("\n🔬 CPU 负载通过 0x202 心跳字段获取，请在 CAN 周期测量中一并记录。")


if __name__ == "__main__":
    main()
