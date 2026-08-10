// canalystii.hpp — 创芯 CANalyst-II (Microchip 04d8:0053) 的 C++ / libusb-1.0 用户态驱动
//
// 移植自 Angus Gratton 的 python-canalystii（Apache-2.0）on-the-wire 协议，字节级一致：
//   端点：命令 EP[ch] = {2,4}，报文 EP[ch] = {1,3}（IN = OUT|0x80）
//   包结构：Message=21B，MessageBuffer=64B（count + Message[3]），命令包=64B
//   初始化：INIT(0x1)+START(0x2)；500k 位定时 BTR0=0x00 BTR1=0x1C
//   发送：MessageBuffer bulk 写报文 EP；接收：MESSAGE_STATUS(0x0A) 读 rx_pending 后 bulk 读
//
// 无 SocketCAN 内核驱动，故直接 libusb。需 -lusb-1.0。非 root 访问需 udev 规则(plugdev)。
#ifndef CANALYSTII_HPP_
#define CANALYSTII_HPP_

#include <libusb-1.0/libusb.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace canalystii {

constexpr uint16_t kVendorId = 0x04D8;
constexpr uint16_t kProductId = 0x0053;

// 命令/报文端点（下标=通道 0/1；CAN2 = 通道 1）
constexpr uint8_t kCommandEp[2] = {0x02, 0x04};
constexpr uint8_t kMessageEp[2] = {0x01, 0x03};

// 命令操作码
constexpr uint32_t kCmdInit = 0x1;
constexpr uint32_t kCmdStart = 0x2;
constexpr uint32_t kCmdStop = 0x3;
constexpr uint32_t kCmdClearRx = 0x5;
constexpr uint32_t kCmdMessageStatus = 0x0A;

#pragma pack(push, 1)
struct Message {
  uint32_t can_id;
  uint32_t timestamp;
  int8_t time_flag;
  int8_t send_type;
  uint8_t remote;
  uint8_t extended;
  int8_t data_len;
  uint8_t data[8];
};
struct MessageBuffer {
  int8_t count;
  Message messages[3];
};
struct SimpleCommand {
  uint32_t command;
  uint32_t padding[0x10 - 0x01];
};
struct InitCommand {
  uint32_t command;
  uint32_t acc_code;
  uint32_t acc_mask;
  uint32_t unknown0;
  uint32_t filter;
  uint32_t unknown1;
  uint32_t timing0;
  uint32_t timing1;
  uint32_t mode;
  uint32_t unknown2;
  uint32_t padding[0x10 - 0x0A];
};
struct MessageStatusResponse {
  uint32_t command;
  uint32_t rx_pending;
  uint16_t tx_pending;
  uint16_t unknown;
  uint32_t padding[0x10 - 0x03];
};
#pragma pack(pop)

static_assert(sizeof(Message) == 0x15, "Message must be 21 bytes");
static_assert(sizeof(MessageBuffer) == 0x40, "MessageBuffer must be 64 bytes");
static_assert(sizeof(SimpleCommand) == 0x40, "SimpleCommand must be 64 bytes");
static_assert(sizeof(InitCommand) == 0x40, "InitCommand must be 64 bytes");
static_assert(sizeof(MessageStatusResponse) == 0x40, "status resp must be 64 bytes");

// 收到的一条 CAN 报文（精简）
struct RxFrame {
  uint32_t can_id;
  uint8_t dlc;
  uint8_t data[8];
  uint32_t timestamp;  // 单位 100us
};

// 位定时表（bitrate → BTR0,BTR1），与 python-canalystii TIMINGS 一致
inline bool timing_for(uint32_t bitrate, uint32_t& t0, uint32_t& t1) {
  switch (bitrate) {
    case 1000000: t0 = 0x00; t1 = 0x14; return true;
    case 800000:  t0 = 0x00; t1 = 0x16; return true;
    case 500000:  t0 = 0x00; t1 = 0x1C; return true;
    case 250000:  t0 = 0x01; t1 = 0x1C; return true;
    case 125000:  t0 = 0x03; t1 = 0x1C; return true;
    case 100000:  t0 = 0x04; t1 = 0x1C; return true;
    case 50000:   t0 = 0x09; t1 = 0x1C; return true;
    case 20000:   t0 = 0x18; t1 = 0x1C; return true;
    case 10000:   t0 = 0x31; t1 = 0x1C; return true;
    default: return false;
  }
}

class Device {
 public:
  Device() = default;
  ~Device() { close(); }
  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;

  // 打开第 device_index 个 CANalyst-II
  void open(int device_index = 0) {
    if (libusb_init(&ctx_) != 0) throw std::runtime_error("libusb_init failed");
    libusb_device** list = nullptr;
    ssize_t n = libusb_get_device_list(ctx_, &list);
    int found = 0;
    libusb_device* target = nullptr;
    for (ssize_t i = 0; i < n; ++i) {
      libusb_device_descriptor desc{};
      if (libusb_get_device_descriptor(list[i], &desc) != 0) continue;
      if (desc.idVendor == kVendorId && desc.idProduct == kProductId) {
        if (found == device_index) { target = libusb_ref_device(list[i]); }
        ++found;
      }
    }
    if (list) libusb_free_device_list(list, 1);
    if (!target) {
      close();
      throw std::runtime_error("No Canalyst-II device (04d8:0053) at index " +
                               std::to_string(device_index));
    }
    int rc = libusb_open(target, &handle_);
    libusb_unref_device(target);
    if (rc != 0 || !handle_) {
      close();
      throw std::runtime_error(std::string("libusb_open failed: ") + libusb_error_name(rc));
    }
    libusb_set_auto_detach_kernel_driver(handle_, 1);
    int cfg = 0;
    if (libusb_get_configuration(handle_, &cfg) != 0 || cfg != 1)
      libusb_set_configuration(handle_, 1);
    rc = libusb_claim_interface(handle_, 0);
    if (rc != 0) {
      close();
      throw std::runtime_error(std::string("claim_interface failed: ") + libusb_error_name(rc) +
                               " (需 udev 规则或权限)");
    }
  }

  void close() {
    if (handle_) {
      libusb_release_interface(handle_, 0);
      libusb_close(handle_);
      handle_ = nullptr;
    }
    if (ctx_) {
      libusb_exit(ctx_);
      ctx_ = nullptr;
    }
  }

  // 初始化并启动一个通道（channel 0=CAN1,1=CAN2）
  void init_channel(int channel, uint32_t bitrate) {
    uint32_t t0 = 0, t1 = 0;
    if (!timing_for(bitrate, t0, t1))
      throw std::runtime_error("unsupported bitrate " + std::to_string(bitrate));
    InitCommand init{};
    init.command = kCmdInit;
    init.acc_code = 0x1;
    init.acc_mask = 0xFFFFFFFF;
    init.filter = 0x1;
    init.timing0 = t0;
    init.timing1 = t1;
    init.mode = 0x0;
    init.unknown2 = 0x1;
    write_ep(kCommandEp[channel], &init, sizeof(init));
    SimpleCommand start{};
    start.command = kCmdStart;
    write_ep(kCommandEp[channel], &start, sizeof(start));
    started_[channel] = true;
  }

  void stop(int channel) {
    if (!handle_ || !started_[channel]) return;
    SimpleCommand cmd{};
    cmd.command = kCmdStop;
    try { write_ep(kCommandEp[channel], &cmd, sizeof(cmd)); } catch (...) {}
    started_[channel] = false;
  }

  // 发送最多 3 条报文（一个 USB 缓冲）。data 为 8 字节标准帧。
  void send(int channel, uint32_t can_id, const uint8_t* data8) {
    MessageBuffer buf{};
    buf.count = 1;
    fill_msg(buf.messages[0], can_id, data8);
    write_ep(kMessageEp[channel], &buf, sizeof(buf));
  }

  // 批量发送任意条数（内部按每 3 条一个缓冲打包）
  void send_batch(int channel, const std::vector<std::pair<uint32_t, const uint8_t*>>& msgs) {
    size_t idx = 0;
    while (idx < msgs.size()) {
      MessageBuffer buf{};
      int c = 0;
      for (; c < 3 && idx < msgs.size(); ++c, ++idx)
        fill_msg(buf.messages[c], msgs[idx].first, msgs[idx].second);
      buf.count = static_cast<int8_t>(c);
      write_ep(kMessageEp[channel], &buf, sizeof(buf));
    }
  }

  // 非阻塞收取当前通道待收报文（返回条数）
  int receive(int channel, std::vector<RxFrame>& out, int timeout_ms = 2) {
    SimpleCommand status_cmd{};
    status_cmd.command = kCmdMessageStatus;
    write_ep(kCommandEp[channel], &status_cmd, sizeof(status_cmd));
    unsigned char resp[0x40];
    int got = read_ep(kCommandEp[channel] | 0x80, resp, sizeof(resp), timeout_ms);
    if (got < static_cast<int>(sizeof(MessageStatusResponse))) return 0;
    MessageStatusResponse status{};
    std::memcpy(&status, resp, sizeof(status));
    if (status.rx_pending == 0) return 0;

    int rx_buffer_num = (static_cast<int>(status.rx_pending) + 2) / 3 + 1;
    int rx_size = rx_buffer_num * static_cast<int>(sizeof(MessageBuffer));
    std::vector<unsigned char> data(rx_size);
    int n = read_ep(kMessageEp[channel] | 0x80, data.data(), rx_size, timeout_ms);
    if (n <= 0) return 0;
    int num_buffers = n / static_cast<int>(sizeof(MessageBuffer));
    int count = 0;
    for (int b = 0; b < num_buffers; ++b) {
      MessageBuffer mb{};
      std::memcpy(&mb, data.data() + b * sizeof(MessageBuffer), sizeof(MessageBuffer));
      int cnt = mb.count;
      if (cnt < 0 || cnt > 3) continue;
      for (int i = 0; i < cnt; ++i) {
        RxFrame f{};
        f.can_id = mb.messages[i].can_id;
        f.dlc = static_cast<uint8_t>(mb.messages[i].data_len);
        f.timestamp = mb.messages[i].timestamp;
        std::memcpy(f.data, mb.messages[i].data, 8);
        out.push_back(f);
        ++count;
      }
    }
    return count;
  }

 private:
  static void fill_msg(Message& m, uint32_t can_id, const uint8_t* data8) {
    m.can_id = can_id;
    m.timestamp = 0;
    m.time_flag = 1;   // 与 python-can canalystii 发送侧一致
    m.send_type = 0;
    m.remote = 0;
    m.extended = 0;    // 标准 11 位帧
    m.data_len = 8;
    std::memcpy(m.data, data8, 8);
  }

  void write_ep(uint8_t ep, const void* data, int len) {
    int transferred = 0;
    int rc = libusb_bulk_transfer(handle_, ep,
                                  reinterpret_cast<unsigned char*>(const_cast<void*>(data)),
                                  len, &transferred, 200);
    if (rc != 0 || transferred != len)
      throw std::runtime_error(std::string("bulk write EP 0x") + hex(ep) + " failed: " +
                               libusb_error_name(rc));
  }

  int read_ep(uint8_t ep, unsigned char* buf, int len, int timeout_ms) {
    int transferred = 0;
    int rc = libusb_bulk_transfer(handle_, ep, buf, len, &transferred, timeout_ms);
    if (rc == LIBUSB_ERROR_TIMEOUT) return transferred;  // 容忍超时，返回已收
    if (rc != 0) return -1;
    return transferred;
  }

  static std::string hex(uint8_t v) {
    char b[3];
    std::snprintf(b, sizeof(b), "%02X", v);
    return std::string(b);
  }

  libusb_context* ctx_ = nullptr;
  libusb_device_handle* handle_ = nullptr;
  bool started_[2] = {false, false};
};

}  // namespace canalystii

#endif  // CANALYSTII_HPP_
