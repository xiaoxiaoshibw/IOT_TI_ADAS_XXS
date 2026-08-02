// orin_shadow_sender.cpp — Jetson 侧「影子/冗余」控制发送进程（C++ / libusb，性能拉满版）
//
// 主控制链（C++ adas_can_gateway → SocketCAN can0，source_id=SRC_PRIMARY，帧 0x10x）之外，
// 再起一个**独立原生进程**做冗余，以 **SRC_BACKUP（帧 0x11x）** 身份，经
// **创芯 CANalyst-II 的 CAN2（channel 1）** 把控制帧发到 MCU 所在的 500k 总线（板载收发器 J14）。
// MCU(safety.c) 对主/备做新鲜度+健康度仲裁，主链失效可无缝切本备源——硬件级热冗余。
//
// 与 Python 版 orin_shadow_sender.py 行为等价，但：纯 C++、无 GIL/GC、steady_clock 定拍、
// 可选 SCHED_FIFO 实时调度 + 内存锁定，适合工业硬实时。
//
// 编译（Jetson/aarch64）：
//   g++ -O2 -std=c++17 -I../include orin_shadow_sender.cpp -o orin_shadow -lusb-1.0 -lpthread
// 运行：
//   ./orin_shadow --self-test                 # 打印 CRC 自检向量
//   ./orin_shadow --source standby --dry-run   # 不开设备，打印一周期帧
//   ./orin_shadow --source standby             # 真发：CAN2 待机备源（enable 全关）
//   ./orin_shadow --source demo --duration 10  # 真发：运动样例（会动执行器）
//   sudo ./orin_shadow --source standby --rt    # 实时调度（SCHED_FIFO）

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sched.h>
#include <sys/mman.h>

#include "orin_can_encoder.hpp"
#include "canalystii.hpp"

using adas_can::AdasCommand;
using adas_can::crc8;
using adas_can::Frame;
using adas_can::OrinCanEncoder;
using Clock = std::chrono::steady_clock;

static std::atomic<bool> g_run{true};
static void on_sigint(int) { g_run.store(false); }

static const char* src_name(uint8_t s) {
  switch (s) {
    case SRC_NONE: return "NONE";
    case SRC_PRIMARY: return "PRIMARY";
    case SRC_BACKUP: return "BACKUP";
    case SRC_WATCHDOG: return "WATCHDOG";
    default: return "?";
  }
}

// ---- 命令来源（冗余计算） ---------------------------------------------------- //
static AdasCommand cmd_standby() {
  AdasCommand c;  // 默认 enable 全关、目标 0
  c.system_mode = SYS_MODE_STANDBY;
  c.soc_health = 1;
  c.fault_level = FAULT_LEVEL_INFO;
  c.control_authority = true;
  c.status_word = ST_PERCEPTION_VALID | ST_LOCALIZATION_VALID | ST_PLANNING_VALID;
  return c;
}

static AdasCommand cmd_demo() {
  AdasCommand c;
  c.target_steer_deg = 6.5f;
  c.target_steer_rate_dps = 300.0f;
  c.lateral_enable = true;
  c.lka_active = true;
  c.target_accel_ms2 = 1.2f;
  c.target_speed_ms = 13.9f;
  c.longitudinal_enable = true;
  c.acc_active = true;
  c.drive_dir = DIR_DRIVE;
  c.status_word = ST_CONTROL_ENABLE | ST_LATERAL_ENABLE | ST_LONGITUDINAL_ENABLE |
                  ST_LKA_ACTIVE | ST_ACC_ACTIVE | ST_PERCEPTION_VALID |
                  ST_LOCALIZATION_VALID | ST_PLANNING_VALID;
  c.system_mode = SYS_MODE_ACTIVE;
  return c;
}

// ---- MCU 心跳(0x202)解码 + CRC 校验 ----------------------------------------- //
struct McuHeartbeat {
  uint8_t state, source, safety_flags, fault_level, seq, alive, load;
};
static bool decode_mcu_heartbeat(const canalystii::RxFrame& f, McuHeartbeat& hb) {
  if (f.can_id != CANID_MCU_HEARTBEAT || f.dlc != 8) return false;
  uint8_t prot[CAN_FRAME_DLC + 1];
  prot[0] = static_cast<uint8_t>(f.can_id & 0xFF);
  prot[1] = static_cast<uint8_t>((f.can_id >> 8) & 0xFF);
  std::memcpy(&prot[2], f.data, CAN_FRAME_DLC - 1);
  if (crc8(prot, CAN_FRAME_DLC + 1) != f.data[CAN_FRAME_DLC - 1]) return false;
  hb.state = f.data[MHB_B_STATE];
  hb.source = f.data[MHB_B_SOURCE];
  hb.safety_flags = f.data[MHB_B_SAFEFLAGS];
  hb.fault_level = f.data[MHB_B_FAULT_LEVEL];
  hb.seq = f.data[MHB_B_SEQ];
  hb.alive = f.data[MHB_B_ALIVE];
  hb.load = f.data[MHB_B_LOAD];
  return true;
}

static void print_frame(const Frame& f) {
  std::printf("  TX 0x%03X ", f.id);
  for (unsigned i = 0; i < CAN_FRAME_DLC; ++i) std::printf(" %02X", f.data[i]);
  std::printf("\n");
}

static void try_realtime() {
  sched_param sp{};
  sp.sched_priority = 80;
  if (sched_setscheduler(0, SCHED_FIFO, &sp) == 0) {
    mlockall(MCL_CURRENT | MCL_FUTURE);
    std::printf("[shadow] 实时调度已启用：SCHED_FIFO prio=80 + mlockall\n");
  } else {
    std::printf("[shadow] ⚠ 实时调度启用失败（需 root 或 rtprio 上限）；以普通调度继续\n");
  }
}

int main(int argc, char** argv) {
  std::string source = "standby";
  int channel = 1;       // CAN2
  uint32_t bitrate = 500000;
  double rate_hz = 100.0;
  double duration = -1.0;  // <0 表示无限
  bool dry_run = false, self_test = false, rt = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--source") source = next();
    else if (a == "--channel") channel = std::atoi(next());
    else if (a == "--bitrate") bitrate = static_cast<uint32_t>(std::atol(next()));
    else if (a == "--rate") rate_hz = std::atof(next());
    else if (a == "--duration") duration = std::atof(next());
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--self-test") self_test = true;
    else if (a == "--rt") rt = true;
    else if (a == "-h" || a == "--help") {
      std::printf("用法: %s [--source standby|demo] [--channel 1] [--bitrate 500000]"
                  " [--rate 100] [--duration N] [--dry-run] [--rt] [--self-test]\n", argv[0]);
      return 0;
    }
  }

  if (self_test) {
    uint8_t v[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    std::printf("CRC-8 自检 crc8(\"123456789\") = 0x%02X\n", crc8(v, 9));
    return 0;
  }

  AdasCommand (*build_command)() = cmd_standby;
  if (source == "demo") {
    std::printf("[shadow] demo 模式：⚠ 运动样例（6.5°/13.9m/s，enable 开）→ 备帧 0x11x\n");
    build_command = cmd_demo;
  } else if (source == "standby") {
    std::printf("[shadow] standby 模式：安全待机（enable 全关，目标 0）→ 备帧 0x11x\n");
  } else {
    std::fprintf(stderr, "未知 --source '%s'（可选 standby|demo）\n", source.c_str());
    return 2;
  }

  OrinCanEncoder enc(SRC_BACKUP);  // ★ 备源 → 帧基址 0x110
  std::printf("[shadow] 目标 CANalyst-II CAN%d(channel %d) @ %u bps → J14；"
              "source_id=SRC_BACKUP，帧基址 0x%03X\n",
              channel + 1, channel, bitrate, enc.base());

  if (dry_run) {
    AdasCommand c = build_command();
    Frame f[4];
    enc.encode_all(c, f);
    for (int i = 0; i < 4; ++i) print_frame(f[i]);
    return 0;
  }

  canalystii::Device dev;
  try {
    dev.open(0);
    dev.init_channel(channel, bitrate);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[shadow] 打开 CANalyst-II 失败: %s\n", e.what());
    return 1;
  }
  if (rt) try_realtime();
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  const auto t0 = Clock::now();
  auto next = t0;
  auto last_report = t0;
  auto last_rx = t0;
  uint64_t tick = 0, tx_frames = 0, tx_errors = 0;
  bool mcu_seen = false;
  McuHeartbeat hb{};
  std::vector<canalystii::RxFrame> rx;

  while (g_run.load()) {
    const auto now = Clock::now();
    if (duration >= 0.0 &&
        std::chrono::duration<double>(now - t0).count() >= duration)
      break;

    AdasCommand cmd = build_command();
    // 组帧：10ms 横向+纵向；20ms 心跳+ADAS（f[] 需在 send_batch 期间存活）
    Frame f[4];
    int nf = 0;
    f[nf++] = enc.build_lateral(cmd);
    f[nf++] = enc.build_longitudinal(cmd);
    if ((tick & 1) == 0) {
      f[nf++] = enc.build_heartbeat(cmd);
      f[nf++] = enc.build_adas_status(cmd);
    }
    std::vector<std::pair<uint32_t, const uint8_t*>> msgs;
    msgs.reserve(nf);
    for (int i = 0; i < nf; ++i) msgs.emplace_back(f[i].id, f[i].data);
    try {
      dev.send_batch(channel, msgs);
      tx_frames += nf;
    } catch (const std::exception&) {
      ++tx_errors;
    }

    // 每 ~20ms 收一次 MCU 回帧
    if (std::chrono::duration<double>(now - last_rx).count() >= 0.02) {
      rx.clear();
      try {
        dev.receive(channel, rx);
        for (const auto& m : rx)
          if (decode_mcu_heartbeat(m, hb)) mcu_seen = true;
      } catch (const std::exception&) {}
      last_rx = now;
    }

    // 每 1s 打印一次状态
    if (std::chrono::duration<double>(now - last_report).count() >= 1.0) {
      double t = std::chrono::duration<double>(now - t0).count();
      if (mcu_seen) {
        uint8_t sf = hb.safety_flags;
        std::printf("[%6.1fs] MCU src=%s state=%d fault=%d load=%d%% flags=0x%02X"
                    "[PRI_FRESH=%d BAK_FRESH=%d BUZZER=%d DEGRADED=%d ESTOP=%d]"
                    " tx=%llu err=%llu\n",
                    t, src_name(hb.source), hb.state, hb.fault_level, hb.load, sf,
                    !!(sf & SF_PRIMARY_FRESH), !!(sf & SF_BACKUP_FRESH),
                    !!(sf & SF_BUZZER_ON), !!(sf & SF_DEGRADED), !!(sf & SF_ESTOP),
                    (unsigned long long)tx_frames, (unsigned long long)tx_errors);
      } else {
        std::printf("[%6.1fs] MCU: <尚无心跳>  tx=%llu err=%llu\n", t,
                    (unsigned long long)tx_frames, (unsigned long long)tx_errors);
      }
      last_report = now;
    }

    ++tick;
    next += std::chrono::duration_cast<Clock::duration>(period);
    auto sleep_dur = next - Clock::now();
    if (sleep_dur > Clock::duration::zero()) {
      std::this_thread::sleep_for(sleep_dur);
    } else {
      next = Clock::now();  // 落后则重对齐，不追赶
    }
  }

  // 退出前补发几帧安全待机，让 MCU 平滑看到备源转安全态
  std::printf("\n[shadow] 收尾：补发安全待机帧并停通道…\n");
  AdasCommand safe = cmd_standby();
  for (int k = 0; k < 5; ++k) {
    Frame f[4];
    enc.encode_all(safe, f);
    std::vector<std::pair<uint32_t, const uint8_t*>> msgs;
    for (int i = 0; i < 4; ++i) msgs.emplace_back(f[i].id, f[i].data);
    try { dev.send_batch(channel, msgs); } catch (...) {}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  dev.stop(channel);
  dev.close();
  std::printf("[shadow] 结束：tx_frames=%llu tx_errors=%llu\n",
              (unsigned long long)tx_frames, (unsigned long long)tx_errors);
  return 0;
}
