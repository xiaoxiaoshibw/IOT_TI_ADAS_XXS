#!/usr/bin/env python3
"""压力测试：对比 Orin Nano (Linux) vs F280025C (裸机) 的 CAN 帧抖动"""
import statistics, sys, time, subprocess, os
import can

CRC8_POLY = 0x31

def crc8_ok(id_, data):
    crc = 0x00
    for b in [id_ & 0xFF, (id_ >> 8) & 0xFF] + [d & 0xFF for d in data[:7]]:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLY) if (crc & 0x80) else (crc << 1)
    return (crc & 0xFF) == (data[7] & 0xFF)

def collect(ch, duration, label):
    bus = can.Bus(interface='canalystii', channel=ch, bitrate=500000)
    ts_orin = []
    ts_mcu = []
    deadline = time.perf_counter() + duration
    while time.perf_counter() < deadline:
        rx = bus.recv(timeout=0.5)
        if rx is None: continue
        if rx.arbitration_id == 0x101:
            ts_orin.append(time.perf_counter())
        elif rx.arbitration_id == 0x202 and rx.dlc >= 8 and crc8_ok(0x202, list(rx.data)):
            ts_mcu.append(time.perf_counter())
    bus.shutdown()
    print(f"  [{label}] 0x101={len(ts_orin)}帧  0x202={len(ts_mcu)}帧")
    return ts_orin, ts_mcu

def analyze(ts, label, ideal_us):
    if len(ts) < 5:
        print(f"  [{label}] 数据不足")
        return
    deltas = [(ts[i]-ts[i-1])*1e6 for i in range(1,len(ts))]
    clean = [d for d in deltas if d > 1000]
    if not clean: return
    s = sorted(clean)
    n = len(s)
    pct = lambda p: s[min(int(p*n), n-1)]
    mean = statistics.fmean(clean)
    jitter = statistics.pstdev(clean)
    errs = [abs(x - mean) for x in clean]
    # 超时比例：偏离预期间隔 > 20%
    outlier = sum(1 for x in clean if abs(x - ideal_us) > ideal_us * 0.2)
    print(f"  [{label}] n={n}  mean={mean:.0f}µs  σ={jitter:.0f}µs  "
          f"p99={pct(0.99):.0f}µs  max={s[-1]:.0f}µs  "
          f"偏离>20%={outlier}/{n}({100.0*outlier/n:.1f}%)")
    return {"mean": mean, "jitter": jitter, "p99": pct(0.99), "max": s[-1],
            "n": n, "outlier_pct": 100.0*outlier/n}

ORIN_SSH = "sshpass -p yahboom ssh jetson@192.168.100.32"

# === 阶段1: 基线（无压力） ===
print("="*60)
print("阶段1: 基线（Orin Nano 空闲）")
print("="*60)
t1_o, t1_m = collect(1, 10, "基线")
r1_o = analyze(t1_o, "Orin Nano 0x101", 20000)
r1_m = analyze(t1_m, "F280025C 0x202", 20000)

# === 阶段2: Orin Nano 压力 ===
print("\n" + "="*60)
print("阶段2: Orin Nano 满 CPU 压力 (stress --cpu 6)")
print("="*60)
# 启动压力
proc = subprocess.Popen(
    f'{ORIN_SSH} "stress --cpu 6 --timeout 30" 2>/dev/null &',
    shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)
t2_o, t2_m = collect(1, 12, "压力")
r2_o = analyze(t2_o, "Orin Nano 0x101", 20000)
r2_m = analyze(t2_m, "F280025C 0x202", 20000)
time.sleep(2)

# === 阶段3: 恢复 ===
print("\n" + "="*60)
print("阶段3: 恢复（Orin Nano 空闲）")
print("="*60)
t3_o, t3_m = collect(1, 10, "恢复")
r3_o = analyze(t3_o, "Orin Nano 0x101", 20000)
r3_m = analyze(t3_m, "F280025C 0x202", 20000)

# === 汇总对比 ===
print("\n" + "="*60)
print("最终对比")
print("="*60)
print(f"{'设备':<20} {'状态':<10} {'σ(µs)':<10} {'p99(µs)':<12} {'偏离>20%':<10}")
print("-"*60)
for dev, results in [("Orin Nano", [(r1_o, "空闲"), (r2_o, "压力"), (r3_o, "恢复")]),
                     ("F280025C", [(r1_m, "空闲"), (r2_m, "压力"), (r3_m, "恢复")])]:
    for r, state in results:
        if r:
            print(f"{dev:<20} {state:<10} {r['jitter']:<10.0f} {r['p99']:<12.0f} {r['outlier_pct']:<9.1f}%")

# 关键结论
if r1_o and r2_o and r1_m and r2_m:
    o_degradation = r2_o['jitter'] / max(r1_o['jitter'], 1)
    m_degradation = r2_m['jitter'] / max(r1_m['jitter'], 1)
    print(f"\n结论：")
    print(f"  Orin Nano   抖动变化: {r1_o['jitter']:.0f} → {r2_o['jitter']:.0f} µs ({o_degradation:.1f}×)")
    print(f"  F280025C    抖动变化: {r1_m['jitter']:.0f} → {r2_m['jitter']:.0f} µs ({m_degradation:.1f}×)")
    print(f"  → F280025C 裸机实时性在压力下 {'不受影响 ✓' if m_degradation < 1.5 else '变化较大'}")
    print(f"  → Orin Nano Linux 调度在压力下 {'显著恶化' if o_degradation > 2 else '略有变化'}")
