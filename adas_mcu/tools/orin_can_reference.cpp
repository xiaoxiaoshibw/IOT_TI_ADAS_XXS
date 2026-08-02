// orin_can_reference.cpp — Orin Nano 侧 CAN 编码参考（联调打印 + SocketCAN 真发样例）
//
// 编码器已抽到 orin_can_encoder.hpp（与 orin_shadow_sender.cpp 共用同一份契约）。
// 本文件只保留：帧打印、SocketCAN(can0) 发送样例、demo 命令、main 自测。
//
// 与 Python 版 orin_can_reference.py、MCU 解码器 src/can_comm.c 三方【字节级一致】。
//
// 编译（Jetson Orin Nano/Linux 真发）：
//   g++ -std=c++14 -I../include orin_can_reference.cpp -o orin_can_ref
//   sudo ip link set can0 up type can bitrate 500000
//   ./orin_can_ref can0            # 往 can0 发一组样例帧（主路 0x10x）
//   ./orin_can_ref                 # 不接硬件：仅打印各帧十六进制（自测字节）
//
// 注意：Jetson Orin Nano HIL 主链路是 PEAK PCAN-USB 适配器 → SocketCAN
//   can1 @ 500k（→ MCU CAN2，主路 0x10x）。板载 mttcan 占 can0 但焊盘未引出、
//   物理不可用（排针 PIN8/PIN10 是 UART 非 CAN，2026-07-19 实测）。
//   CANalyst-II 的 CAN2（备路 0x11x）走 libusb，见 orin_shadow_sender.cpp。

#include <cstdint>
#include <cstdio>

#include "orin_can_encoder.hpp"  // 共享编码器（-I../include 取 adas_can_protocol.h）

using adas_can::AdasCommand;
using adas_can::crc8;
using adas_can::Frame;
using adas_can::OrinCanEncoder;

// ------------------------------------------------------------------ //
// 打印 / 发送
// ------------------------------------------------------------------ //
static void print_frame(const Frame& f) {
  std::printf("  0x%03X ", f.id);
  for (uint16_t i = 0; i < CAN_FRAME_DLC; ++i) std::printf(" %02X", f.data[i]);
  std::printf("\n");
}

#if defined(__linux__)
#include <cstring>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// 打开一个 SocketCAN 接口（如 Jetson Orin Nano 上的 "can0"），失败返回 -1
static int can_open(const char* ifname) {
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) { perror("socket"); return -1; }
  struct ifreq ifr; std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); return -1; }
  struct sockaddr_can addr; std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); return -1; }
  return s;
}

static bool send_frame(int s, const Frame& f) {
  struct can_frame cf; std::memset(&cf, 0, sizeof(cf));
  cf.can_id = f.id;  // 标准 11 位 ID（不置 CAN_EFF_FLAG）
  cf.can_dlc = CAN_FRAME_DLC;
  std::memcpy(cf.data, f.data, CAN_FRAME_DLC);
  return write(s, &cf, sizeof(cf)) == (ssize_t)sizeof(cf);
}
#endif

static AdasCommand demo_cruise() {
  AdasCommand c;
  c.target_steer_deg = 6.5f; c.target_steer_rate_dps = 300.0f;
  c.lateral_enable = true; c.lka_active = true;
  c.target_accel_ms2 = 1.2f; c.target_speed_ms = 13.9f;
  c.longitudinal_enable = true; c.acc_active = true; c.drive_dir = DIR_DRIVE;
  c.status_word = ST_CONTROL_ENABLE | ST_LATERAL_ENABLE | ST_LONGITUDINAL_ENABLE |
                  ST_LKA_ACTIVE | ST_ACC_ACTIVE | ST_PERCEPTION_VALID |
                  ST_LOCALIZATION_VALID | ST_PLANNING_VALID;
  c.system_mode = SYS_MODE_ACTIVE;
  return c;
}

int main(int argc, char** argv) {
  OrinCanEncoder enc(SRC_PRIMARY);
  AdasCommand cmd = demo_cruise();
  Frame frames[4];
  enc.encode_all(cmd, frames);

#if defined(__linux__)
  if (argc > 1) {  // 传接口名 → 真发
    int s = can_open(argv[1]);
    if (s < 0) return 1;
    for (int i = 0; i < 4; ++i) send_frame(s, frames[i]);
    std::printf("已向 %s 发送 4 条控制帧。\n", argv[1]);
    close(s);
    return 0;
  }
#else
  (void)argc; (void)argv;
#endif

  std::printf("周期样例帧（Orin 主路 → MCU）：\n");
  for (int i = 0; i < 4; ++i) print_frame(frames[i]);

  uint8_t chk[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  std::printf("\nCRC-8 自检: crc8(\"123456789\") = 0x%02X\n", crc8(chk, 9));
  return 0;
}
