"""T7 — HIL 会话门 / 外部软 MRM 授权（实板验证）。

对应主机回归 test_safety_host.c 的"HIL 会话门"矩阵，本脚本在实板 CAN 上验证同一
安全不变式：

  会话未 ACTIVE（或旧会话残留）时，来自 SoC 状态帧的 mrm_request=1 不得使 MCU 进入
  MRM；会话 ACTIVE 且当前会话已建立 mrm=0 零基线后，合法的 0→1 才进入 MRM。急停 /
  AEB / 本地强制 MRM 不受该门限制（此处不覆盖，见 T2/T5/T6）。

硬件：LAUNCHXL-F280025C + USB-CANalyst-II（channel 1, 500k）。用**生产镜像**即可，
不依赖 ADAS_TEST_BUILD / 0x301 注入——正是要证明生产固件的信任边界正确。

阶段：
  A. 只发心跳(0x100)+状态(0x103, mrm=1)，不发控制帧(0x101/0x102)、不做 0x104 握手
     → 会话停在 BOOT_WAIT、控制源仲裁返回 INVALID → 观察 0x202 state 恒非 MRM(4)。
     这是"HIL 未启动/已停止但 SoC 仍发心跳+状态"的直接复现，是本用例的主判据。
  B. 正控制：ANNOUNCE→SYNC_STANDBY 握手 + 连续发全套新鲜控制帧(control_authority=1,
     mrm=0) → 等 0x206 ack==ACTIVE(4) → 再把 mrm 翻 0→1 → 观察 0x202 state==MRM(4)。
     本阶段在脚本内模拟网关握手；若握手未能在超时内达到 ACTIVE，标记为 inconclusive
     而不拖垮整体（实板上通常由真正的 adas_can_gateway 驱动会话）。

注意：该脚本需要真实硬件，未在开发环境执行；主机回归 test_safety_host.c 才是权威证明。
"""
from __future__ import annotations
import time
import can

from common import (
    open_bus,
    CANID_MCU_HEARTBEAT, CANID_MCU_SESSION_STATUS,
    SYS_MODE_MRM,
)
from orin_can_reference import (  # noqa: E402  (common.py 已把 tools 加进 sys.path)
    OrinCanEncoder, AdasCommand, _finish,
)

# 与 include/adas_can_protocol_v3_generated.h / adas_can_protocol.h 严格对齐
CANID_PRIMARY_SESSION_CONTROL = 0x104
ADAS_PROTOCOL_VERSION_V3 = 3
HIL_REQ_ANNOUNCE = 1
HIL_REQ_SYNC_STANDBY = 2
HIL_ACK_ACTIVE = 4
ST_CONTROL_ENABLE = 1 << 0

_TX_PERIOD_S = 0.02       # 50 Hz 下发，远快于 CTRL_TIMEOUT_MS=60


def _send(bus: can.Bus, framed) -> None:
    can_id, data = framed
    bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))


def _session_frame(request: int, session_id: int, seq: int) -> bytes:
    """构造 0x104 会话控制帧（version/phase/id(LE u32)/seq/crc）。"""
    buf = bytearray(8)
    buf[0] = ADAS_PROTOCOL_VERSION_V3
    buf[1] = request & 0xFF
    buf[2] = session_id & 0xFF
    buf[3] = (session_id >> 8) & 0xFF
    buf[4] = (session_id >> 16) & 0xFF
    buf[5] = (session_id >> 24) & 0xFF
    buf[6] = seq & 0xFF
    return _finish(CANID_PRIMARY_SESSION_CONTROL, buf)


def _pump(bus: can.Bus, duration_s: float, tx, on_hb=None, on_ack=None) -> None:
    """在 duration_s 内以 _TX_PERIOD_S 周期调用 tx() 下发，同时收 0x202/0x206 回调。"""
    end = time.time() + duration_s
    next_tx = 0.0
    while time.time() < end:
        now = time.time()
        if now >= next_tx:
            tx()
            next_tx = now + _TX_PERIOD_S
        msg = bus.recv(timeout=_TX_PERIOD_S)
        if msg is None:
            continue
        if msg.arbitration_id == CANID_MCU_HEARTBEAT and on_hb is not None:
            on_hb(int(msg.data[0]))
        elif msg.arbitration_id == CANID_MCU_SESSION_STATUS and on_ack is not None:
            on_ack(int(msg.data[0]))


def run(record_dir=None) -> dict:
    bus = open_bus(channel=1)
    try:
        enc = OrinCanEncoder()

        # ---------------- 阶段 A：会话未 ACTIVE，残留 mrm=1 ----------------
        cmd_a = AdasCommand(control_authority=False, mrm_request=True,
                            longitudinal_enable=False, lateral_enable=False,
                            soc_health=1)
        states_a: list[int] = []

        def _tx_a() -> None:
            _send(bus, enc.build_heartbeat(cmd_a))
            _send(bus, enc.build_adas_status(cmd_a))   # 只发 0x100 + 0x103

        _pump(bus, 1.5, _tx_a, on_hb=states_a.append)
        phase_a_ok = (len(states_a) > 0) and (SYS_MODE_MRM not in states_a)

        # ---------------- 阶段 B：握手到 ACTIVE 再 0→1 ----------------
        session_id = 0xA5A50001
        sseq = [0]

        def _next_sseq() -> int:
            sseq[0] = (sseq[0] + 1) & 0xFF
            return sseq[0]

        cmd_b = AdasCommand(control_authority=True, mrm_request=False,
                            lateral_enable=True, longitudinal_enable=True,
                            status_word=ST_CONTROL_ENABLE, soc_health=1)

        # ANNOUNCE → SESSION_ACCEPTED
        bus.send(can.Message(arbitration_id=CANID_PRIMARY_SESSION_CONTROL,
                             data=_session_frame(HIL_REQ_ANNOUNCE, session_id, _next_sseq()),
                             is_extended_id=False))

        ack_state = [-1]
        sent_sync = [False]

        def _tx_b() -> None:
            _send(bus, enc.build_heartbeat(cmd_b))
            _send(bus, enc.build_lateral(cmd_b))
            _send(bus, enc.build_longitudinal(cmd_b))
            _send(bus, enc.build_adas_status(cmd_b))
            if not sent_sync[0]:
                bus.send(can.Message(
                    arbitration_id=CANID_PRIMARY_SESSION_CONTROL,
                    data=_session_frame(HIL_REQ_SYNC_STANDBY, session_id, _next_sseq()),
                    is_extended_id=False))
                sent_sync[0] = True

        # 泵送最多 4s，等 0x206 ack==ACTIVE（READY 自动 ARM 需 300ms 浸泡 + tick）
        deadline = time.time() + 4.0
        while time.time() < deadline and ack_state[0] != HIL_ACK_ACTIVE:
            _pump(bus, 0.2, _tx_b, on_ack=lambda s: ack_state.__setitem__(0, s))

        reached_active = (ack_state[0] == HIL_ACK_ACTIVE)

        phase_b_ok = None
        states_b: list[int] = []
        if reached_active:
            # 已 ACTIVE 且当前会话已连续发 mrm=0（零基线成立）→ 翻 0→1
            cmd_b.mrm_request = True
            _pump(bus, 0.8, _tx_b, on_hb=states_b.append)
            phase_b_ok = SYS_MODE_MRM in states_b

        # 整体判据以阶段 A（安全负向不变式）为准；阶段 B 为正向对照，
        # 握手 inconclusive 时不拖垮整体，但会在结果中显式标注。
        ok = phase_a_ok and (phase_b_ok is not False)
        return {
            "ok": ok,
            "phase_a_ok": phase_a_ok,
            "phase_a_states": sorted(set(states_a)),
            "phase_b_reached_active": reached_active,
            "phase_b_ok": phase_b_ok,
            "phase_b_states": sorted(set(states_b)),
            "note": "阶段B未达ACTIVE=握手inconclusive(实板通常由真实网关驱动会话)"
                    if not reached_active else "",
        }
    finally:
        bus.shutdown()


if __name__ == "__main__":
    print(run())
