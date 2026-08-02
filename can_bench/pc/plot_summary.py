#!/usr/bin/env python3
"""抖动对比汇总图表"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

stages = ["Idle", "High Load\n(2000 f/s)", "Ultra Load\n(5000 f/s)"]
orin_jitter = [257, 1048, 293]
mcu_jitter  = [263, 775, 290]
orin_p99    = [22006, 21683, 22000]
mcu_p99     = [22040, 21999, 22001]

x = np.arange(len(stages))
w = 0.3

# 左图: 抖动 σ
ax1.bar(x - w/2, orin_jitter, w, label="Orin Nano (Linux+ROS2)", color="#e41a1c", alpha=0.85)
ax1.bar(x + w/2, mcu_jitter,  w, label="F280025C (Bare-metal RT)", color="#377eb8", alpha=0.85)
for i in range(len(stages)):
    ax1.text(i - w/2, orin_jitter[i] + 20, f"{orin_jitter[i]}µs", ha='center', va='bottom', fontsize=9, color="#e41a1c", fontweight='bold')
    ax1.text(i + w/2, mcu_jitter[i] + 20, f"{mcu_jitter[i]}µs", ha='center', va='bottom', fontsize=9, color="#377eb8", fontweight='bold')
ax1.set_ylabel("Jitter sigma (us)")
ax1.set_title("Frame Interval Jitter (sigma) - lower is better")
ax1.set_xticks(x)
ax1.set_xticklabels(stages)
ax1.legend(fontsize=9)
ax1.grid(True, alpha=0.3, axis='y')

# 右图: p99
ax2.bar(x - w/2, orin_p99, w, label="Orin Nano (Linux+ROS2)", color="#e41a1c", alpha=0.85)
ax2.bar(x + w/2, mcu_p99,  w, label="F280025C (Bare-metal RT)", color="#377eb8", alpha=0.85)
for i in range(len(stages)):
    ax2.text(i - w/2, orin_p99[i] + 20, f"{orin_p99[i]}µs", ha='center', va='bottom', fontsize=9, color="#e41a1c", fontweight='bold')
    ax2.text(i + w/2, mcu_p99[i] + 20, f"{mcu_p99[i]}µs", ha='center', va='bottom', fontsize=9, color="#377eb8", fontweight='bold')
ax2.set_ylabel("p99 Interval (us)")
ax2.set_title("Tail Latency p99 - lower is better")
ax2.set_xticks(x)
ax2.set_xticklabels(stages)
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3, axis='y')

fig.suptitle("F280025C vs Orin Nano - CAN Frame Timing Determinism\n"
             "(F280025C: full ADAS safety stack | Orin Nano: Linux + ROS2)",
             fontsize=13, fontweight='bold')

fig.text(0.5, 0.01,
    "F280025C runs a complete safety stack (control + monitoring + diagnostics) while maintaining CAN timing\n"
    "jitter comparable to or better than Orin Nano. Under high bus load, the bare-metal polling advantage is clear.",
    ha='center', fontsize=11, style='italic')

fig.tight_layout(rect=[0, 0.06, 1, 0.93])
fig.savefig("/home/xxs/ADAS_ORIN_TI/can_benchmark/pc/f280025c_vs_orin_comparison.png", dpi=150)
print("已保存 f280025c_vs_orin_comparison.png")
