#!/usr/bin/env python3
"""
ADAS HIL 证据分析 & 图表生成工具
================================
读取 evidence/ 目录下的原始 CSV 数据，生成竞赛文档用图表。

用法:
  python3 analyze_evidence.py                              # 分析所有已有证据
  python3 analyze_evidence.py --output-dir ../../文档/ADAS报告修订包/图/
"""

import argparse
import csv
import json
import os
import sys
from collections import defaultdict
from datetime import datetime

# 确保 matplotlib 可用（无 GUI 后端用 Agg）
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# 中文字体
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'Noto Sans CJK SC',
                                    'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['figure.figsize'] = (10, 6)

# ============================================================================
# 分析函数
# ============================================================================

def analyze_frame_period(csv_path: str, output_dir: str):
    """分析 0x201/0x202 CAN 反馈帧到达周期"""
    print(f"\n{'='*50}")
    print(f"📊 分析帧周期数据: {csv_path}")

    data_201 = []  # delta_ms for 0x201
    data_202 = []  # delta_ms for 0x202
    timestamps_201 = []
    timestamps_202 = []

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row['t_s'])
            d = float(row['delta_ms'])
            can_id = row['can_id'].strip()
            if can_id == '0x201':
                data_201.append(d)
                timestamps_201.append(t)
            elif can_id == '0x202':
                data_202.append(d)
                timestamps_202.append(t)

    def stats(arr, label):
        if not arr:
            print(f"  {label}: 无数据")
            return None
        arr = np.array(arr)
        s = {
            'label': label,
            'n': len(arr),
            'mean': float(np.mean(arr)),
            'std': float(np.std(arr)),
            'min': float(np.min(arr)),
            'p50': float(np.percentile(arr, 50)),
            'p95': float(np.percentile(arr, 95)),
            'p99': float(np.percentile(arr, 99)),
            'max': float(np.max(arr)),
        }
        print(f"  {label}: n={s['n']}, mean={s['mean']:.3f}ms, "
              f"σ={s['std']:.3f}, p95={s['p95']:.3f}, p99={s['p99']:.3f}, "
              f"max={s['max']:.3f}")
        return s

    s201 = stats(data_201, '0x201 (控制反馈, 10ms设计)')
    s202 = stats(data_202, '0x202 (心跳, 20ms设计)')

    # ---- 图1: 周期分布直方图 ----
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    for ax, arr, title, color, design_val in [
        (axes[0], data_201, '0x201 控制反馈帧到达周期', '#3b82f6', 10.0),
        (axes[1], data_202, '0x202 心跳帧到达周期', '#22c55e', 20.0)
    ]:
        if not arr:
            continue
        arr_np = np.array(arr)
        bins = np.linspace(max(0, design_val - 5), design_val + 5, 40)
        ax.hist(arr_np, bins=bins, color=color, alpha=0.7, edgecolor='white', linewidth=0.5)
        ax.axvline(design_val, color='red', linestyle='--', linewidth=1.5, label=f'设计值 {design_val}ms')
        ax.axvline(np.mean(arr_np), color='orange', linestyle='-', linewidth=1.5, label=f'均值 {np.mean(arr_np):.3f}ms')
        ax.set_xlabel('周期 (ms)')
        ax.set_ylabel('频次')
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(output_dir, 'fig_frame_period_histogram.png')
    plt.savefig(out_path)
    plt.close()
    print(f"  → 已保存: {out_path}")

    # ---- 图2: 时序散点图（观察抖动） ----
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    for ax, t, d, title, color, design_val in [
        (axes[0], timestamps_201, data_201, '0x201 周期时序', '#3b82f6', 10.0),
        (axes[1], timestamps_202, data_202, '0x202 周期时序', '#22c55e', 20.0)
    ]:
        if not t:
            continue
        ax.scatter(t, d, s=1, color=color, alpha=0.5)
        ax.axhline(design_val, color='red', linestyle='--', linewidth=1, label=f'设计值 {design_val}ms')
        ax.set_ylabel('周期 (ms)')
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[1].set_xlabel('时间 (s)')
    plt.tight_layout()
    out_path = os.path.join(output_dir, 'fig_frame_period_timeseries.png')
    plt.savefig(out_path)
    plt.close()
    print(f"  → 已保存: {out_path}")

    # ---- 图3: CDF 累积分布 ----
    fig, ax = plt.subplots(figsize=(10, 5))

    for arr, label, color in [
        (data_201, '0x201 控制反馈 (设计10ms)', '#3b82f6'),
        (data_202, '0x202 心跳 (设计20ms)', '#22c55e')
    ]:
        if not arr:
            continue
        sorted_data = np.sort(arr)
        cdf = np.arange(1, len(sorted_data) + 1) / len(sorted_data)
        ax.plot(sorted_data, cdf * 100, label=label, color=color, linewidth=2)
        ax.axvline(np.percentile(sorted_data, 95), color=color, linestyle=':', linewidth=1,
                    alpha=0.7)
        ax.axvline(np.percentile(sorted_data, 99), color=color, linestyle='--', linewidth=1,
                    alpha=0.7)

    ax.set_xlabel('到达周期 (ms)')
    ax.set_ylabel('累积概率 (%)')
    ax.set_title('CAN 反馈帧到达周期 CDF')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(8, 24)

    plt.tight_layout()
    out_path = os.path.join(output_dir, 'fig_frame_period_cdf.png')
    plt.savefig(out_path)
    plt.close()
    print(f"  → 已保存: {out_path}")

    return s201, s202


def analyze_cpu_load(csv_path: str, output_dir: str):
    """分析 MCU CPU 自报负载"""
    print(f"\n{'='*50}")
    print(f"📊 分析 CPU 负载数据: {csv_path}")

    loads = []
    states = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            loads.append(int(row['loop_load_pct']))
            states.append(int(row['state']))

    loads_np = np.array(loads)
    print(f"  loop_load_pct: n={len(loads)}, mean={np.mean(loads_np):.2f}%, "
          f"min={np.min(loads_np)}%, max={np.max(loads_np)}%, "
          f"p99={np.percentile(loads_np, 99):.0f}%")

    # 负载分布直方图
    fig, ax = plt.subplots(figsize=(10, 5))
    bins = np.arange(0, 25, 1)
    ax.hist(loads, bins=bins, color='#f97316', alpha=0.7, edgecolor='white', linewidth=0.5)
    ax.axvline(np.mean(loads_np), color='red', linestyle='--', linewidth=2,
               label=f'均值 {np.mean(loads_np):.2f}%')
    ax.axvline(np.percentile(loads_np, 99), color='orange', linestyle=':',
               linewidth=2, label=f'P99 {np.percentile(loads_np, 99):.0f}%')
    ax.set_xlabel('loop_load_pct (%)')
    ax.set_ylabel('频次')
    ax.set_title('MCU 自报 CPU 负载分布')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(output_dir, 'fig_cpu_load_histogram.png')
    plt.savefig(out_path)
    plt.close()
    print(f"  → 已保存: {out_path}")

    return {
        'n': len(loads),
        'mean': float(np.mean(loads_np)),
        'min': int(np.min(loads_np)),
        'max': int(np.max(loads_np)),
        'p95': float(np.percentile(loads_np, 95)),
        'p99': float(np.percentile(loads_np, 99)),
    }


def analyze_hil_session(csv_dir: str, output_dir: str):
    """分析 HIL 会话状态推进"""
    print(f"\n{'='*50}")
    print(f"📊 分析 HIL 会话数据: {csv_dir}")

    session_files = [
        'evidence_hil_session_coldboot_raw.csv',
        'evidence_hil_session_longwatch_raw.csv',
        'evidence_hil_session_raw.csv',
    ]

    results = {}
    for fname in session_files:
        fpath = os.path.join(csv_dir, fname)
        if not os.path.exists(fpath):
            print(f"  ⚠ 跳过不存在的文件: {fname}")
            continue

        print(f"\n  📄 {fname}:")
        with open(fpath) as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            print(f"    记录数: {len(rows)}")

            # 检查会话阶段
            if 'phase' in rows[0]:
                phases = [r['phase'] for r in rows]
                phase_counts = defaultdict(int)
                for p in phases:
                    phase_counts[p] += 1
                print(f"    会话阶段分布: {dict(phase_counts)}")

            if 'session_id' in rows[0]:
                session_ids = set(r['session_id'].strip() for r in rows)
                print(f"    会话 ID 数: {len(session_ids)}")

            results[fname] = {
                'rows': len(rows),
                'fields': list(rows[0].keys()),
            }

    return results


def generate_evidence_summary(evidence_dir: str, output_dir: str):
    """生成证据汇总报告"""
    print(f"\n{'='*50}")
    print(f"📊 生成证据汇总报告")

    summary = {
        'generated_at': datetime.now().isoformat(),
        'evidence_dir': evidence_dir,
        'files': {},
    }

    for fname in sorted(os.listdir(evidence_dir)):
        fpath = os.path.join(evidence_dir, fname)
        if os.path.isfile(fpath):
            size_kb = os.path.getsize(fpath) / 1024
            summary['files'][fname] = {
                'size_kb': round(size_kb, 1),
                'type': 'csv' if fname.endswith('.csv') else
                        'png' if fname.endswith('.png') else
                        'py' if fname.endswith('.py') else
                        'out' if fname.endswith('.out') else
                        'md' if fname.endswith('.md') else 'other',
            }

    out_path = os.path.join(output_dir, 'evidence_summary.json')
    with open(out_path, 'w') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"  → 已保存: {out_path}")

    # 打印摘要
    print(f"\n  证据文件 ({len(summary['files'])} 个):")
    for fname, info in sorted(summary['files'].items()):
        print(f"    {fname}  ({info['size_kb']} KB, {info['type']})")

    return summary


# ============================================================================
# 主入口
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='ADAS HIL 证据分析工具')
    parser.add_argument('--evidence-dir', default=None,
                        help='evidence 目录路径（默认自动查找）')
    parser.add_argument('--output-dir', default=None,
                        help='输出目录（默认 evidence 同级）')
    args = parser.parse_args()

    # 自动查找 evidence 目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..'))

    evidence_dir = args.evidence_dir
    if not evidence_dir:
        candidates = [
            os.path.join(repo_root, '文档', 'ADAS报告修订包', 'evidence'),
        ]
        for c in candidates:
            if os.path.isdir(c):
                evidence_dir = c
                break

    if not evidence_dir or not os.path.isdir(evidence_dir):
        print(f"❌ 找不到 evidence 目录，请用 --evidence-dir 指定")
        sys.exit(1)

    print(f"📁 证据目录: {evidence_dir}")

    output_dir = args.output_dir
    if not output_dir:
        output_dir = os.path.join(evidence_dir, '..', '图', '分析图表')
    os.makedirs(output_dir, exist_ok=True)
    print(f"📂 输出目录: {output_dir}")
    print(f"⏰ 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    # 1. 分析帧周期
    fp_path = os.path.join(evidence_dir, 'evidence_frame_period_raw.csv')
    if os.path.exists(fp_path):
        s201, s202 = analyze_frame_period(fp_path, output_dir)

    # 2. 分析 CPU 负载
    cpu_path = os.path.join(evidence_dir, 'evidence_cpu_load_raw.csv')
    if os.path.exists(cpu_path):
        cpu_stats = analyze_cpu_load(cpu_path, output_dir)

    # 3. 分析 HIL 会话
    session_results = analyze_hil_session(evidence_dir, output_dir)

    # 4. 生成汇总报告
    summary = generate_evidence_summary(evidence_dir, output_dir)

    print(f"\n{'='*50}")
    print("✅ 分析完成！")
    print(f"📂 所有图表已保存到: {output_dir}")
    print(f"{'='*50}")


if __name__ == '__main__':
    main()
