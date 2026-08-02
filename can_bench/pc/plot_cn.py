#!/usr/bin/env python3
"""ESP32 vs F280025C 帧间隔抖动对比图（中文）"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_prop = fm.FontProperties(fname=font_path)
plt.rcParams['font.family'] = font_prop.get_name()
plt.rcParams['axes.unicode_minus'] = False

import numpy as np

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

labels = ["ESP32\n(FreeRTOS echo)", "F280025C\n(裸机安全栈)"]
sigma  = [838, 318]
p99    = [21889, 22391]
colors = ["#d73027", "#1a9850"]
x = np.arange(2)
w = 0.35

ax1.bar(x, sigma, w, color=colors, alpha=0.85, edgecolor='white', linewidth=1.5)
for i, v in enumerate(sigma):
    ax1.text(i, v + 15, f"{v} µs", ha='center', fontsize=13, fontweight='bold', color=colors[i])
ax1.set_ylabel("抖动 σ (µs)")
ax1.set_title("帧间隔抖动 — 越小越确定", fontsize=13, fontweight='bold')
ax1.set_xticks(x); ax1.set_xticklabels(labels, fontsize=11)
ax1.grid(True, alpha=0.3, axis='y')
ax1.set_ylim(0, 1200)

ax2.bar(x, p99, w, color=colors, alpha=0.85, edgecolor='white', linewidth=1.5)
for i, v in enumerate(p99):
    ax2.text(i, v + 100, f"{v} µs", ha='center', fontsize=13, fontweight='bold', color=colors[i])
ax2.set_ylabel("p99 帧间隔 (µs)")
ax2.set_title("尾部延迟 — 越低越可靠", fontsize=13, fontweight='bold')
ax2.set_xticks(x); ax2.set_xticklabels(labels, fontsize=11)
ax2.grid(True, alpha=0.3, axis='y')
ax2.set_ylim(0, 26000)

fig.suptitle("ESP32 vs F280025C — CAN 帧间隔抖动对比", fontsize=15, fontweight='bold')
fig.text(0.5, 0.01,
    "F280025C 运行完整 ADAS 安全栈（5路CAN收发+控制运算+安全监控+诊断），抖动仅 318µs；\n"
    "ESP32 仅运行单一 echo 任务，抖动却达 838µs（F280025C 的 2.6 倍），充分体现裸机实时控制器的确定性优势。",
    ha='center', fontsize=11, style='italic')
fig.tight_layout(rect=[0, 0.08, 1, 0.95])
fig.savefig("/home/xxs/ADAS_ORIN_TI/can_benchmark/pc/esp32_vs_f280025c_cn.png", dpi=150)
print("已保存 esp32_vs_f280025c_cn.png")
