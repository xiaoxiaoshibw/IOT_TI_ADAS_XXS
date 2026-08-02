#!/usr/bin/env python3
"""
bench.py — CANalyst-II 双通道 MCU 对比基准
             CH0 (CAN1) → ESP32 应答器      CH1 (CAN2) → F280025C(C28x) 应答器

对两颗 MCU 施加**完全相同**的 CAN 激励（同帧格式 / 同 CRC / 同计算负载），
量化对比四项指标，突出 F280025C 的实时确定性与计算吞吐优势：

  T1 响应延迟&抖动    固定间隔发 0x301，测往返 RTT 的 均值/最小/最大/p99/抖动(σ)
  T2 计算负载下延迟    请求携带 load 档，设备跑同样多的控制数学后再回，测 RTT-under-load
  T3 最大无丢帧吞吐    逐档抬升发帧速率，找各自"零丢帧"的最高持续帧率
  T4 中断响应确定性    T1 的 RTT 分布直方图 / CDF（确定性一眼可见）

RTT 测法：send() 后取 perf_counter 为 t0，阻塞 recv 到配对的 0x302 取 t1，RTT=t1-t0。
绝对值含一段**恒定**的 USB/驱动开销（两通道对称），因此**设备间差值**才是信号——
这一点在 README 里有说明，报告里也据此表述。

用法：
  python3 bench.py all                      # 跑全部四项，两通道
  python3 bench.py t1 --count 5000
  python3 bench.py t2 --loads 0,1,2,4,8
  python3 bench.py t3 --rates 1000,2000,5000,8000,10000
  python3 bench.py all --esp-channel 0 --mcu-channel 1 --plot report.png
  python3 bench.py t1 --only mcu           # 只测某一颗（另一颗没接时）
依赖：python-can[canalystii]、可选 matplotlib（--plot 时才需要）。
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass, field

import can  # python-can

from canframe import (
    BENCH_REQ_ID,
    build_request,
    BenchResponse,
    DEVICE_NAMES,
)


# ----------------------------- 通道封装 -----------------------------
# CANalyst-II 是单个 USB 设备的两个通道：底层 canalystii 库每次 open() 都会独占
# claim 整个 USB 接口，两个 Target 各自 can.Bus() 会在第二个上撞成 Resource busy。
# 正确用法是一个 CANalystIIBus(channel=[0,1]) 共享给两侧，但共享后 recv() 返回的
# 帧不再天然按 Target 分流（且两侧应答 ID 完全相同，是设计上的公平点），所以这里
# 用每个 Target 自己的 deque 做一层按 channel 路由，避免互相"偷帧"污染结果。
from collections import deque as _deque


@dataclass
class Target:
    name: str          # "ESP32" / "F280025C"
    channel: int       # canalystii 通道号
    bus: can.BusABC = None
    _shared: bool = False
    _inbox: _deque = field(default_factory=_deque)

    def open(self, bitrate: int, shared_bus=None):
        if shared_bus is not None:
            self.bus = shared_bus
            self._shared = True
        else:
            self.bus = can.Bus(interface="canalystii", channel=self.channel, bitrate=bitrate)

    def close(self):
        if self.bus is not None and not self._shared:
            self.bus.shutdown()
        self.bus = None

    def _pump(self, timeout: float):
        """从共享总线拉一帧；本通道的放进自己 inbox，别的通道的放进它的 inbox。"""
        rx = self.bus.recv(timeout=timeout)
        if rx is None:
            return False
        owner = _TARGETS_BY_CHANNEL.get(getattr(rx, "channel", self.channel), self)
        owner._inbox.append(rx)
        return True

    def flush(self):
        """清掉残留 RX，避免上一子测试的响应污染本次配对。"""
        self._inbox.clear()
        if not self._shared:
            deadline = time.perf_counter() + 0.05
            while time.perf_counter() < deadline:
                if self.bus.recv(timeout=0.0) is None:
                    break
            return
        deadline = time.perf_counter() + 0.05
        while time.perf_counter() < deadline:
            if not self._pump(timeout=0.0):
                break
        self._inbox.clear()

    def _recv_own(self, timeout: float):
        if self._inbox:
            return self._inbox.popleft()
        if not self._shared:
            return self.bus.recv(timeout=timeout)
        deadline = time.perf_counter() + timeout
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return None
            self._pump(timeout=remaining)
            if self._inbox:
                return self._inbox.popleft()

    def ping(self, seq: int, load_units: int, timeout: float):
        """发一帧请求，等配对的 0x302，返回 (rtt_s | None, BenchResponse | None)。"""
        req = build_request(seq=seq, load_units=load_units)
        msg = can.Message(arbitration_id=BENCH_REQ_ID, is_extended_id=False, data=req,
                          channel=self.channel if self._shared else None)
        self.bus.send(msg)
        t0 = time.perf_counter()
        deadline = t0 + timeout
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return None, None
            rx = self._recv_own(remaining)
            if rx is None:
                return None, None
            if BenchResponse.is_response(rx):
                resp = BenchResponse(rx.data)
                if resp.seq == (seq & 0xFF):
                    return time.perf_counter() - t0, resp
                # 序号不匹配（迟到的旧响应）→ 继续等


_TARGETS_BY_CHANNEL: dict = {}  # channel -> Target，供 _pump 路由使用


# ----------------------------- 统计 -----------------------------
@dataclass
class LatStats:
    target: str
    load_units: int = 0
    rtts_us: list = field(default_factory=list)
    sent: int = 0
    lost: int = 0
    crc_bad: int = 0

    def summary(self) -> dict:
        r = sorted(self.rtts_us)
        n = len(r)
        pct = lambda p: r[min(n - 1, int(p * n))] if n else float("nan")
        return {
            "target": self.target,
            "load": self.load_units,
            "n": n,
            "sent": self.sent,
            "lost": self.lost,
            "loss_pct": 100.0 * self.lost / self.sent if self.sent else 0.0,
            "min": r[0] if n else float("nan"),
            "mean": statistics.fmean(r) if n else float("nan"),
            "p50": pct(0.50),
            "p99": pct(0.99),
            "max": r[-1] if n else float("nan"),
            "jitter": statistics.pstdev(r) if n > 1 else 0.0,
        }


def _fmt_row(s: dict) -> str:
    return (f"  {s['target']:<10} n={s['n']:<5} "
            f"min={s['min']:7.1f}  mean={s['mean']:7.1f}  p50={s['p50']:7.1f}  "
            f"p99={s['p99']:7.1f}  max={s['max']:8.1f}  σ={s['jitter']:7.1f} µs  "
            f"loss={s['loss_pct']:.2f}%")


def _print_compare(title: str, per_target: dict):
    print(f"\n=== {title} ===")
    summaries = {t: st.summary() for t, st in per_target.items()}
    for s in summaries.values():
        print(_fmt_row(s))
    names = list(summaries)
    if "F280025C" in names and "ESP32" in names:
        f, e = summaries["F280025C"], summaries["ESP32"]
        if f["mean"] and e["mean"]:
            print(f"  → F280025C 均值延迟 {e['mean']/f['mean']:.1f}× 更低，"
                  f"抖动(σ) {e['jitter']/max(f['jitter'],1e-9):.1f}× 更小，"
                  f"尾延迟 p99 {e['p99']/max(f['p99'],1e-9):.1f}× 更低")
    return summaries


# ----------------------------- 子测试 -----------------------------
def run_latency(targets, count, interval_ms, load_units, timeout) -> dict:
    per = {t.name: LatStats(t.name, load_units) for t in targets}
    interval = interval_ms / 1000.0
    for i in range(count):
        for t in targets:
            st = per[t.name]
            st.sent += 1
            rtt, resp = t.ping(seq=i, load_units=load_units, timeout=timeout)
            if rtt is None:
                st.lost += 1
            elif not resp.crc_ok:
                st.crc_bad += 1
            else:
                st.rtts_us.append(rtt * 1e6)
        if interval:
            time.sleep(interval)
    return per


def run_throughput(targets, rates, dwell_s, timeout) -> dict:
    """
    逐档以目标帧率连发（不等每帧响应），并行统计回帧数；
    某档丢帧率 > 0.1% 即视为超过该设备无损吞吐上限。
    """
    results = {t.name: [] for t in targets}
    for t in targets:
        print(f"\n[T3] {t.name} 吞吐扫描:")
        max_lossless = 0
        for rate in rates:
            t.flush()
            period = 1.0 / rate
            n = max(1, int(rate * dwell_s))
            seqs_sent = set()
            t_next = time.perf_counter()
            for i in range(n):
                t.bus.send(can.Message(arbitration_id=BENCH_REQ_ID, is_extended_id=False,
                                       data=build_request(seq=i, load_units=0),
                                       channel=t.channel if t._shared else None))
                seqs_sent.add(i & 0xFF)  # 注意 8-bit 回绕，仅近似
                t_next += period
                slack = t_next - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
            # 收尾：把在途响应收干净
            got = 0
            deadline = time.perf_counter() + 0.2 + timeout
            while time.perf_counter() < deadline:
                rx = t._recv_own(timeout)
                if rx is None:
                    break
                if BenchResponse.is_response(rx):
                    got += 1
            loss_pct = 100.0 * (n - min(got, n)) / n
            ok = loss_pct <= 0.1
            if ok:
                max_lossless = rate
            print(f"    {rate:6d} f/s  发 {n:6d}  回 {got:6d}  丢 {loss_pct:6.2f}%  "
                  f"{'OK' if ok else '✗ 超限'}")
            results[t.name].append((rate, got, n, loss_pct))
            if not ok:
                break
        print(f"    → {t.name} 无丢帧最高持续帧率 ≈ {max_lossless} f/s")
    return results


# ----------------------------- 绘图 -----------------------------
def make_plots(path, t1_summaries, t1_raw, t2_summaries, t3_results):
    import matplotlib
    matplotlib.use("Agg")
    # 默认字体不含中文字形，标题/坐标轴会渲染成方块；找一个系统里已装的中文
    # 字体挂上去（Noto Sans CJK SC 优先，其次文泉驿），避免图里出现乱码方框。
    import matplotlib.font_manager as fm
    for _name in ("Noto Sans CJK SC", "WenQuanYi Zen Hei", "AR PL UMing CN"):
        if any(_name == f.name for f in fm.fontManager.ttflist):
            matplotlib.rcParams["font.sans-serif"] = [_name]
            break
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    # 分类色板固定顺序 slot1/slot2（蓝/橙），可通过色觉障碍相邻对校验
    colors = {"F280025C": "#2a78d6", "ESP32": "#eb6834"}

    # T1/T4: RTT 直方图（对数纵轴看尾部）
    ax = axes[0][0]
    for name, st in t1_raw.items():
        if st.rtts_us:
            ax.hist(st.rtts_us, bins=80, alpha=0.6, label=name,
                    color=colors.get(name), log=True)
    ax.set_title("T1/T4 往返延迟分布 (log)")
    ax.set_xlabel("RTT (µs)"); ax.set_ylabel("帧数"); ax.legend()

    # T4: CDF
    ax = axes[0][1]
    for name, st in t1_raw.items():
        r = sorted(st.rtts_us)
        if r:
            ax.plot(r, [i / len(r) for i in range(len(r))], label=name,
                    color=colors.get(name))
    ax.set_title("T4 RTT 累积分布 (CDF) — 越陡越确定")
    ax.set_xlabel("RTT (µs)"); ax.set_ylabel("累积占比"); ax.legend()

    # T2: 计算负载 vs 均值 RTT
    ax = axes[1][0]
    for name in t2_summaries:
        xs = sorted(t2_summaries[name])
        ax.plot(xs, [t2_summaries[name][l]["mean"] for l in xs], "-o",
                label=name, color=colors.get(name))
    ax.set_title("T2 计算负载 vs 均值延迟")
    ax.set_xlabel("负载档 (×256 次控制数学迭代)"); ax.set_ylabel("RTT (µs)"); ax.legend()

    # T3: 吞吐
    ax = axes[1][1]
    for name, rows in t3_results.items():
        xs = [r[0] for r in rows]
        loss = [r[3] for r in rows]
        ax.plot(xs, loss, "-s", label=name, color=colors.get(name))
    ax.axhline(0.1, ls="--", c="gray", lw=1, label="0.1% 阈值")
    ax.set_title("T3 发帧率 vs 丢帧率")
    ax.set_xlabel("目标帧率 (f/s)"); ax.set_ylabel("丢帧 %"); ax.legend()

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"\n[图] 已保存 {path}")


# ----------------------------- main -----------------------------
def main():
    ap = argparse.ArgumentParser(description="CANalyst-II 双通道 MCU 对比基准")
    ap.add_argument("test", choices=["t1", "t2", "t3", "t4", "all"])
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--esp-channel", type=int, default=0, help="CAN1→ESP32 通道号")
    ap.add_argument("--mcu-channel", type=int, default=1, help="CAN2→F280025C 通道号")
    ap.add_argument("--only", choices=["esp", "mcu"], help="只测一颗")
    ap.add_argument("--count", type=int, default=2000, help="T1/T2 每档帧数")
    ap.add_argument("--interval-ms", type=float, default=5.0, help="T1/T2 发帧间隔")
    ap.add_argument("--timeout", type=float, default=0.05, help="单帧响应超时(s)")
    ap.add_argument("--loads", default="0,1,2,4,8", help="T2 负载档，逗号分隔")
    ap.add_argument("--rates", default="1000,2000,4000,6000,8000,10000",
                    help="T3 帧率档，逗号分隔")
    ap.add_argument("--dwell", type=float, default=1.0, help="T3 每档持续秒数")
    ap.add_argument("--plot", metavar="PNG", help="输出对比图(需 matplotlib)")
    ap.add_argument("--raw-csv", metavar="DIR",
                    help="把T1原始RTT样本(每目标一个文件)存成CSV，供复核/重新画图")
    args = ap.parse_args()

    targets = []
    if args.only != "mcu":
        targets.append(Target("ESP32", args.esp_channel))
    if args.only != "esp":
        targets.append(Target("F280025C", args.mcu_channel))

    for t in targets:
        _TARGETS_BY_CHANNEL[t.channel] = t

    try:
        if len(targets) > 1:
            # 单个 CANalystIIBus 同时驱动两个通道，避免两个 can.Bus() 各自
            # claim 同一 USB 设备导致第二个 Resource busy。
            shared = can.Bus(interface="canalystii",
                             channel=[t.channel for t in targets],
                             bitrate=args.bitrate)
            for t in targets:
                t.open(args.bitrate, shared_bus=shared)
        else:
            for t in targets:
                t.open(args.bitrate)
    except Exception as e:
        print(f"[错误] 打开 CAN 总线失败: {e}", file=sys.stderr)
        for x in targets:
            x.close()
        return 2
    for t in targets:
        t.flush()

    t1_raw = {}
    t1_sum = {}
    t2_sum = {}
    t3_res = {}
    try:
        if args.test in ("t1", "t4", "all"):
            per = run_latency(targets, args.count, args.interval_ms, 0, args.timeout)
            t1_raw = per
            t1_sum = _print_compare("T1 响应延迟 & 抖动 (load=0)", per)
            if args.raw_csv:
                import csv, os
                os.makedirs(args.raw_csv, exist_ok=True)
                for name, st in per.items():
                    fp = os.path.join(args.raw_csv, f"t1_rtt_us_{name}.csv")
                    with open(fp, "w", newline="") as f:
                        w = csv.writer(f)
                        w.writerow(["rtt_us"])
                        for v in st.rtts_us:
                            w.writerow([f"{v:.3f}"])
                    print(f"[原始数据] {fp} (n={len(st.rtts_us)})")

        if args.test in ("t2", "all"):
            loads = [int(x) for x in args.loads.split(",")]
            t2_sum = {t.name: {} for t in targets}
            for load in loads:
                per = run_latency(targets, max(500, args.count // 2),
                                  args.interval_ms, load, args.timeout)
                s = _print_compare(f"T2 计算负载下延迟 (load={load})", per)
                for name in s:
                    t2_sum[name][load] = s[name]

        if args.test in ("t3", "all"):
            rates = [int(x) for x in args.rates.split(",")]
            t3_res = run_throughput(targets, rates, args.dwell, args.timeout)

        if args.plot and args.test == "all":
            make_plots(args.plot, t1_sum, t1_raw, t2_sum, t3_res)
        elif args.plot:
            print("[提示] --plot 目前仅在 `all` 下汇总四图；单项请自行取 CSV。")
    finally:
        shared_bus = targets[0].bus if targets and targets[0]._shared else None
        for t in targets:
            t.close()
        if shared_bus is not None:
            shared_bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
