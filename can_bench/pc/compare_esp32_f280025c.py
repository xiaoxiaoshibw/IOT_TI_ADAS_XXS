#!/usr/bin/env python3
"""ESP32 vs F280025C 帧间隔抖动对比（分通道采集）"""
import statistics, time, sys, can
from canframe import build_request, BenchResponse, BENCH_REQ_ID

CRC8_POLY = 0x31

def crc8_ok(id_, data):
    crc = 0x00
    for b in [id_ & 0xFF, (id_ >> 8) & 0xFF] + [d & 0xFF for d in data[:7]]:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLY) if (crc & 0x80) else (crc << 1)
    return (crc & 0xFF) == (data[7] & 0xFF)

def collect_esp(duration_s):
    """CH0: 每 20ms 发请求给 ESP32，收 0x302 响应"""
    bus = can.Bus(interface='canalystii', channel=0, bitrate=500000)
    ts = []
    deadline = time.perf_counter() + duration_s
    seq = 0
    next_tick = time.perf_counter()
    while time.perf_counter() < deadline:
        now = time.perf_counter()
        if now >= next_tick:
            bus.send(can.Message(arbitration_id=BENCH_REQ_ID, is_extended_id=False,
                                 data=build_request(seq=seq, load_units=0)))
            seq = (seq + 1) & 0xFF
            next_tick += 0.020
        rx = bus.recv(timeout=0.002)
        if rx and BenchResponse.is_response(rx):
            ts.append(time.perf_counter())
    bus.shutdown()
    return ts

def collect_mcu(duration_s):
    """CH1: 监听 F280025C 0x202 心跳帧"""
    bus = can.Bus(interface='canalystii', channel=1, bitrate=500000)
    ts = []
    deadline = time.perf_counter() + duration_s
    while time.perf_counter() < deadline:
        rx = bus.recv(timeout=0.1)
        if rx and rx.arbitration_id == 0x202 and rx.dlc >= 8 and crc8_ok(0x202, list(rx.data)):
            ts.append(time.perf_counter())
    bus.shutdown()
    return ts

def analyze(ts, label, exp_us):
    if len(ts) < 5: return None
    deltas = [(ts[i]-ts[i-1])*1e6 for i in range(1,len(ts))]
    clean = [d for d in deltas if d > exp_us*0.3 and d < exp_us*1.5]
    if not clean: return None
    s = sorted(clean); n = len(s)
    pct = lambda p: s[min(int(p*n), n-1)]
    mean = statistics.fmean(clean); jit = statistics.pstdev(clean)
    print(f"  [{label}] n={n}  mean={mean:.0f}us  sigma={jit:.0f}us  "
          f"p99={pct(0.99):.0f}us  min={s[0]:.0f}us  max={s[-1]:.0f}us")
    return {"mean": mean, "sigma": jit, "p99": pct(0.99), "min": s[0], "max": s[-1], "n": n, "data": clean}

print("="*60)
print("ESP32 vs F280025C - 帧间隔抖动对比")
print("="*60)

print("\n[1/2] 采集 ESP32 (CH0, PC每20ms发请求→0x302响应)...")
t_esp = collect_esp(30)
r_esp = analyze(t_esp, "ESP32 0x302", 20000)

print("\n[2/2] 采集 F280025C (CH1, 0x202 心跳)...")
t_mcu = collect_mcu(30)
r_mcu = analyze(t_mcu, "F280025C 0x202", 21000)

if r_esp and r_mcu:
    print("\n" + "="*60)
    print("对比汇总")
    print("="*60)
    h = "{:<25} {:>8} {:>10} {:>12} {:>10} {:>10} {:>10}"
    print(h.format("Device", "n", "Mean(us)", "Sigma(us)", "p99(us)", "Min(us)", "Max(us)"))
    print("-"*80)
    for r, name in [(r_esp, "ESP32 (echo task)"), (r_mcu, "F280025C (full safety stack)")]:
        print(h.format(name, r['n'], f"{r['mean']:.0f}", f"{r['sigma']:.0f}", f"{r['p99']:.0f}", f"{r['min']:.0f}", f"{r['max']:.0f}"))
    print(f"\n  ESP32     : 纯 echo 任务, FreeRTOS")
    print(f"  F280025C  : 完整安全栈(控制+监控+诊断), 裸机轮询 @1kHz, CPU仅5%")
    print(f"  抖动比(σ F280025C/ESP32): {r_mcu['sigma']/max(r_esp['sigma'],1):.1f}x")
    if r_mcu['sigma'] <= r_esp['sigma'] * 1.5:
        print("  → F280025C 在运行完整安全栈（5路CAN收发+控制计算+安全监控+诊断）的同时，")
        print("    帧间隔抖动与ESP32（纯echo任务）相当甚至更优，体现裸机实时控制器的确定性优势")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.5))
        colors = {"ESP32": "#d73027", "F280025C": "#1a9850"}
        for ax, (r, name) in [(ax1, (r_esp, "ESP32")), (ax2, (r_mcu, "F280025C"))]:
            d = sorted(r['data']); n = len(d)
            ax.plot(d, [i/n for i in range(n)], color=colors[name], lw=2)
            ax.axvline(r['mean'], color=colors[name], ls='--', alpha=0.6, label=f"mean={r['mean']:.0f}us")
            ax.axvline(r['p99'], color='gray', ls=':', alpha=0.8, label=f"p99={r['p99']:.0f}us")
            ax.set_xlabel("Frame interval (us)"); ax.set_ylabel("CDF")
            ax.set_title(f"{name}\n(RTOS echo)" if name=="ESP32" else f"{name}\n(bare-metal safety stack)")
            ax.legend(fontsize=8, loc='lower right'); ax.grid(True, alpha=0.3)
        fig.suptitle("ESP32 vs F280025C - CAN Frame Interval CDF (~20ms period)", fontsize=13, fontweight='bold')
        fig.tight_layout()
        fig.savefig("/home/xxs/ADAS_ORIN_TI/can_benchmark/pc/esp32_vs_f280025c_cdf.png", dpi=150)
        print("[图] 已保存 esp32_vs_f280025c_cdf.png")
    except Exception as e:
        print(f"[图] 跳过: {e}")
