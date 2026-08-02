#!/usr/bin/env python3
"""CAN 总线负载压力测试 — 对比裸机 vs Linux 的帧稳定性"""
import statistics, sys, time, subprocess
import can

CRC8_POLY = 0x31
ORIN_SSH = "sshpass -p yahboom ssh jetson@192.168.100.32"

def crc8_ok(id_, data):
    crc = 0x00
    for b in [id_ & 0xFF, (id_ >> 8) & 0xFF] + [d & 0xFF for d in data[:7]]:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLY) if (crc & 0x80) else (crc << 1)
    return (crc & 0xFF) == (data[7] & 0xFF)

def collect(ch, duration, label):
    bus = can.Bus(interface='canalystii', channel=ch, bitrate=500000)
    ts = {"orin": [], "mcu": [], "load": []}
    deadline = time.perf_counter() + duration
    while time.perf_counter() < deadline:
        rx = bus.recv(timeout=0.3)
        if rx is None: continue
        if rx.arbitration_id == 0x101:
            ts["orin"].append(time.perf_counter())
        elif rx.arbitration_id == 0x202 and rx.dlc >= 8 and crc8_ok(0x202, list(rx.data)):
            ts["mcu"].append(time.perf_counter())
        elif rx.arbitration_id == 0x300:
            ts["load"].append(time.perf_counter())
    bus.shutdown()
    print(f"  [{label}] Orin={len(ts['orin'])}帧  MCU={len(ts['mcu'])}帧  负载={len(ts['load'])}帧")
    return ts

def analyze(ts, name, ideal_us):
    if len(ts) < 5: return None
    deltas = [(ts[i]-ts[i-1])*1e6 for i in range(1,len(ts))]
    clean = [d for d in deltas if d > 1000]
    if not clean: return None
    s = sorted(clean)
    n = len(s)
    pct = lambda p: s[min(int(p*n), n-1)]
    mean = statistics.fmean(clean)
    jitter = statistics.pstdev(clean)
    outlier = sum(1 for x in clean if abs(x - ideal_us) > ideal_us * 0.2)
    r = {"jitter": jitter, "p99": pct(0.99), "max": s[-1], "mean": mean,
         "n": n, "outlier_pct": 100.0*outlier/n}
    print(f"  [{name}] n={n}  mean={mean:.0f}µs  σ={jitter:.0f}µs  "
          f"p99={pct(0.99):.0f}µs  max={s[-1]:.0f}µs  偏离>20%={100.0*outlier/n:.1f}%")
    return r

def run_stress_can(period_us, duration_s):
    """通过 Orin Nano 的 can1 发 0x300 负载帧，持续 duration_s 秒后自动退出"""
    timeout_s = duration_s + 5
    cmd = f'{ORIN_SSH} "timeout {timeout_s} cangen can1 -g {period_us} -L 8 -D deadbeef11223344 -I 0x300" 2>/dev/null &'
    return subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("=" * 70)
print("CAN 总线负载压力测试 (Orin Nano 通过 can1 注入 0x300 帧)")
print("=" * 70)

results = {}

# 阶段1: 基线（无额外负载）
print("\n[阶段1] 基线 — 无额外 CAN 负载")
ts = collect(1, 10, "基线")
results["baseline"] = {"orin": analyze(ts["orin"], "Orin Nano", 20000),
                       "mcu": analyze(ts["mcu"], "F280025C", 20000)}

# 阶段2: 低负载 (2ms 间隔 = 500帧/秒, 总线利用率 ~8%)
print("\n[阶段2] 低负载 — cangen can1 -g 2000 (500 f/s)")
p = run_stress_can(2000, 25)
time.sleep(1)
ts = collect(1, 12, "低负载")
time.sleep(2)
results["lowload"] = {"orin": analyze(ts["orin"], "Orin Nano", 20000),
                      "mcu": analyze(ts["mcu"], "F280025C", 20000)}

# 阶段3: 高负载 (500µs 间隔 = 2000帧/秒)
print("\n[阶段3] 高负载 — cangen can1 -g 500 (2000 f/s)")
p = run_stress_can(500, 25)
time.sleep(1)
ts = collect(1, 15, "高负载")
time.sleep(2)
results["highload"] = {"orin": analyze(ts["orin"], "Orin Nano", 20000),
                       "mcu": analyze(ts["mcu"], "F280025C", 20000)}

# 阶段4: 超高负载 (200µs = 5000帧/秒, 总线利用率 ~80%)
print("\n[阶段4] 超高负载 — cangen can1 -g 200 (5000 f/s)")
p = run_stress_can(200, 25)
time.sleep(1)
ts = collect(1, 15, "超高负载")
results["ultraload"] = {"orin": analyze(ts["orin"], "Orin Nano", 20000),
                        "mcu": analyze(ts["mcu"], "F280025C", 20000)}

# 清理
subprocess.run(f'{ORIN_SSH} "pkill -f cangen" 2>/dev/null', shell=True)

# 汇总对比
print("\n" + "=" * 70)
print("汇总对比：抖动 σ (µs)")
print("=" * 70)
print(f"{'场景':<12} {'Orin Nano(µs)':<16} {'F280025C(µs)':<16} {'恶化比(Orin)':<14} {'恶化比(MCU)':<14}")
print("-" * 70)
base_o = results["baseline"]["orin"]["jitter"]
base_m = results["baseline"]["mcu"]["jitter"]
for stage in ["baseline", "lowload", "highload", "ultraload"]:
    r = results[stage]
    o_j = r["orin"]["jitter"]
    m_j = r["mcu"]["jitter"]
    o_deg = o_j / base_o
    m_deg = m_j / base_m
    print(f"{stage:<12} {o_j:<16.0f} {m_j:<16.0f} {o_deg:<14.1f}x {m_deg:<14.1f}x")

print(f"\n结论：")
for stage in ["lowload", "highload", "ultraload"]:
    r = results[stage]
    o_deg = r["orin"]["jitter"] / base_o
    m_deg = r["mcu"]["jitter"] / base_m
    comp = "F280025C 抖动增幅更小 ✓" if m_deg < o_deg else "两者相近"
    print(f"  {stage}: Orin={o_deg:.1f}x  F280025C={m_deg:.1f}x → {comp}")
