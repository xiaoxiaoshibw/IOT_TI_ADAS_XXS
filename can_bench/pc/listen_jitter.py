#!/usr/bin/env python3
"""监听 F280025C 0x202 心跳帧，分析帧间隔抖动(σ/jitter)。"""
import statistics
import sys
import time
import can

CAN_ID_HB = 0x202

CRC8_POLY = 0x31

def crc8_frame(id_, data):
    crc = 0x00
    buf = [id_ & 0xFF, (id_ >> 8) & 0xFF] + [d & 0xFF for d in data[:7]]
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLY) if (crc & 0x80) else (crc << 1)
    return crc & 0xFF

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", type=int, default=1, help="CANalyst-II 通道号")
    ap.add_argument("--duration", type=float, default=10.0, help="监听秒数")
    ap.add_argument("--bitrate", type=int, default=500000)
    args = ap.parse_args()

    bus = can.Bus(interface="canalystii", channel=args.channel, bitrate=args.bitrate)
    print(f"监听 F280025C 0x202 心跳 {args.duration}s ...", file=sys.stderr)

    timestamps = []
    seqs = []
    states = []
    loads = []
    crc_pass = 0
    crc_fail = 0

    t_start = time.perf_counter()
    deadline = t_start + args.duration
    while time.perf_counter() < deadline:
        rx = bus.recv(timeout=1.0)
        if rx is None:
            continue
        if rx.arbitration_id != CAN_ID_HB or rx.dlc < 8:
            continue
        d = list(rx.data)
        calc = crc8_frame(CAN_ID_HB, d)
        if calc != d[7]:
            crc_fail += 1
            continue
        crc_pass += 1
        now = time.perf_counter()
        timestamps.append(now)
        seqs.append(d[4] & 0xFF)
        states.append(d[0] & 0xFF)
        loads.append(d[6] & 0xFF)

    bus.shutdown()

    n = len(timestamps)
    if n < 2:
        print(f"收到 {n} 帧有效心跳，不足分析", file=sys.stderr)
        return 1

    deltas = [(timestamps[i] - timestamps[i-1]) * 1e6 for i in range(1, n)]
    d = sorted(deltas)
    pct = lambda p: d[min(int(p * len(d)), len(d)-1)]

    print(f"\n=== F280025C 0x202 心跳间隔抖动 ({n-1} 个间隔, {args.duration}s) ===")
    print(f"  CRC 通过={crc_pass}  失败={crc_fail}")
    print(f"  MCU 状态分布: {dict((s, states.count(s)) for s in set(states))}")
    print(f"  CPU 负载: mean={statistics.fmean(loads):.1f}%  max={max(loads)}%")
    print(f"")
    print(f"  min  = {d[0]:8.2f} µs")
    print(f"  mean = {statistics.fmean(d):8.2f} µs")
    print(f"  p50  = {pct(0.50):8.2f} µs")
    print(f"  p99  = {pct(0.99):8.2f} µs")
    print(f"  max  = {d[-1]:8.2f} µs")
    print(f"  σ    = {statistics.pstdev(d):8.2f} µs  (抖动)")
    ideal = statistics.fmean(d)
    max_dev = max(abs(x - ideal) for x in deltas)
    print(f"  最大偏差 = {max_dev:.2f} µs")
    print(f"  → 预期 20ms({20000}µs)，实测均值 {statistics.fmean(d):.0f}µs = "
          f"{1e6/statistics.fmean(d):.1f} Hz")

    # 按预期间隔 20ms 分类
    near_20ms = sum(1 for x in deltas if abs(x - 20000) < 1000)
    print(f"  → {near_20ms}/{len(deltas)} 个间隔在 19~21ms 范围内 ("
          f"{100.0*near_20ms/len(deltas):.1f}%)")

if __name__ == "__main__":
    sys.exit(main())
