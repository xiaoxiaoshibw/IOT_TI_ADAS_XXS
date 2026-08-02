"""
canframe.py — Benchmark 帧编解码 + CRC-8（与 MCU crc8.c / orin_can_reference.py 一致）

复用 v3 协议的 0x301(请求)/0x302(响应) ID。为避免触发 MCU 侧的故障注入语义，
请求 byte0 固定为 BENCH_MARKER=0xBE —— 该值不在 0..9 的注入命令范围内，
Safety_applyInject() 会将其当作未知命令直接忽略，因此纯粹用作 echo 探针。

帧格式（8 字节，byte7 = CRC-8）：
  请求 0x301 (PC → device)
    byte0 = 0xBE          BENCH_MARKER
    byte1 = seq           0..255 滚动，用于配对与丢帧检测
    byte2 = load_units    计算负载档位（每档 = 256 次控制数学迭代；0 = 纯 echo）
    byte3 = flags         保留
    byte4..6 = 0
    byte7 = CRC-8
  响应 0x302 (device → PC)
    byte0 = 0xBE          原样回显
    byte1 = seq           原样回显
    byte2 = load_units    原样回显
    byte3 = device_id     0xE5=ESP32  0x28=F280025C(C28x)
    byte4..5 = compute_lo 计算核结果低 16 位（防止编译器优化掉 + 校验双方跑了同样的活）
    byte6 = 0
    byte7 = CRC-8

CRC-8：poly 0x31，MSB-first，init 0x00，无反转/无异或输出。
覆盖 [id_low, id_high, byte0..byte6]。
"""
from __future__ import annotations

BENCH_REQ_ID = 0x301
BENCH_RESP_ID = 0x302
BENCH_MARKER = 0xBE

DEVICE_ID_ESP32 = 0xE5
DEVICE_ID_C28X = 0x28
DEVICE_NAMES = {DEVICE_ID_ESP32: "ESP32", DEVICE_ID_C28X: "F280025C"}


def crc8(data: bytes) -> int:
    """CRC-8 / poly 0x31, MSB-first, init 0x00（与 crc8.c 逐位一致）。"""
    crc = 0x00
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc & 0xFF


def crc8_frame(can_id: int, payload7: bytes) -> int:
    """帧 CRC = crc8(id_low, id_high, byte0..byte6)。"""
    header = bytes((can_id & 0xFF, (can_id >> 8) & 0xFF))
    return crc8(header + bytes(payload7[:7]))


def build_request(seq: int, load_units: int = 0, flags: int = 0) -> bytes:
    buf = bytearray(8)
    buf[0] = BENCH_MARKER
    buf[1] = seq & 0xFF
    buf[2] = load_units & 0xFF
    buf[3] = flags & 0xFF
    buf[7] = crc8_frame(BENCH_REQ_ID, buf)
    return bytes(buf)


class BenchResponse:
    __slots__ = ("seq", "load_units", "device_id", "compute_lo", "crc_ok")

    def __init__(self, data: bytes):
        self.seq = data[1]
        self.load_units = data[2]
        self.device_id = data[3]
        self.compute_lo = data[4] | (data[5] << 8)
        self.crc_ok = (len(data) >= 8 and data[7] == crc8_frame(BENCH_RESP_ID, data))

    @property
    def device_name(self) -> str:
        return DEVICE_NAMES.get(self.device_id, f"0x{self.device_id:02X}")

    @staticmethod
    def is_response(msg) -> bool:
        return (getattr(msg, "arbitration_id", None) == BENCH_RESP_ID
                and len(msg.data) >= 8 and msg.data[0] == BENCH_MARKER)


if __name__ == "__main__":
    # 自检：CRC-8 已知向量（与 orin_can_reference.py 一致）
    assert crc8(b"123456789") == 0xA2, hex(crc8(b"123456789"))
    req = build_request(seq=42, load_units=3)
    print("请求样例:", req.hex(" "), "CRC 自检通过")
