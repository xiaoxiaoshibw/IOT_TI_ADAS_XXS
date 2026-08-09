#ifndef ADAS_GUI__PREFLIGHT_CHECK_HPP_
#define ADAS_GUI__PREFLIGHT_CHECK_HPP_

// 启动前一键体检：check_hil_ready.py 是"起来了没起对"（要求栈已经在跑），
// 本模块补的是它管不到的那一段——"起之前，起不起得来"。全部检查项都不需要
// CARLA/桥接已经在跑，可以在点[启动完整系统]之前先跑一遍。
//
// 分级规则（与 hil_common.py 的 FAIL/WARN 分级一致）：
//   Fail — 几乎必定导致启动失败或已知会引发崩溃，阻断启动（LaunchPanel 侧）。
//   Warn — 可能有问题但不阻断，用户确认后可继续。
//   Ok   — 正常。
//
// 分类函数（classify_*）是纯逻辑，无系统调用，gtest 覆盖；run_preflight()
// 做实际的文件/网络/USB 探测，只在 GUI 运行期调用。

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QString>
#include <QVector>

#include "dds_env.hpp"
#include "launch_config.hpp"

namespace adas::gui {

enum class PreflightLevel { Ok, Warn, Fail };

struct PreflightItem {
  QString id;
  QString label;
  PreflightLevel level{PreflightLevel::Ok};
  QString detail;
};

inline QString preflight_level_name(PreflightLevel level) {
  switch (level) {
    case PreflightLevel::Ok: return QStringLiteral("OK");
    case PreflightLevel::Warn: return QStringLiteral("WARN");
    case PreflightLevel::Fail: return QStringLiteral("FAIL");
  }
  return QStringLiteral("?");
}

// ===== 纯逻辑（gtest 覆盖）=====

// CARLA 可执行文件是否存在于配置的 CARLA 目录下。
inline PreflightItem classify_carla_executable(bool exists, const QString& path) {
  return {QStringLiteral("carla_exe"), QStringLiteral("CARLA 可执行文件"),
          exists ? PreflightLevel::Ok : PreflightLevel::Fail,
          exists ? path : QStringLiteral("未找到 %1，请检查「CARLA 目录」设置").arg(path)};
}

// 非 NVIDIA 显卡 + Epic 画质 + 有本地渲染窗口 是今天实测复现 3 次的崩溃组合
// （LowLevelFatalError close: Bad file descriptor → Signal 11，均发生在刚
// 加载地图后几秒内；复用已运行 CARLA 实例的会话从未崩溃）。命中即 Warn，
// 不设为 Fail：某些 NVIDIA/AMD 环境经验证是稳定的，不应一刀切阻断。
inline PreflightItem classify_gpu_quality(const QString& gpu_vendor, bool low_quality,
                                           bool render_offscreen) {
  const bool is_nvidia = gpu_vendor.contains(QStringLiteral("NVIDIA"), Qt::CaseInsensitive);
  const bool mitigated = low_quality || render_offscreen;
  if (!is_nvidia && !mitigated) {
    return {QStringLiteral("gpu_quality"), QStringLiteral("显卡/画质匹配"), PreflightLevel::Warn,
            QStringLiteral("显卡=%1，当前 Epic 画质+本地渲染窗口是今天实测崩溃过 3 次的组合，"
                           "建议勾选「低画质」或「离屏渲染」")
                .arg(gpu_vendor.isEmpty() ? QStringLiteral("未知/非NVIDIA") : gpu_vendor)};
  }
  return {QStringLiteral("gpu_quality"), QStringLiteral("显卡/画质匹配"), PreflightLevel::Ok,
          QStringLiteral("显卡=%1，画质设置无已知风险组合")
              .arg(gpu_vendor.isEmpty() ? QStringLiteral("未知") : gpu_vendor)};
}

inline PreflightItem classify_orin_reachable(bool reachable, const QString& peer) {
  return {QStringLiteral("orin_ping"), QStringLiteral("Orin 网络可达"),
          reachable ? PreflightLevel::Ok : PreflightLevel::Fail,
          reachable ? QStringLiteral("%1 ping 通").arg(peer)
                    : QStringLiteral("%1 ping 不通，检查直连网线/网卡").arg(peer)};
}

inline PreflightItem classify_direct_iface(const QString& iface) {
  return {QStringLiteral("direct_iface"), QStringLiteral("直连网卡"),
          iface.isEmpty() ? PreflightLevel::Warn : PreflightLevel::Ok,
          iface.isEmpty()
              ? QStringLiteral("未探测到 192.168.100.x 网段网卡，DDS 将退化为默认组播发现，"
                               "可能出现「发现得到、收不到数据」")
              : QStringLiteral("%1（192.168.100.x 网段）").arg(iface)};
}

// PC 侧 CANalyst-II 仅在真实 CAN HIL（control_source != ros2）时需要；
// ros2 桌面闭环不经 MCU/CAN，检不到设备也不该阻断。
inline PreflightItem classify_can_adapter(const QString& control_source, bool device_present) {
  if (control_source == QStringLiteral("ros2")) {
    return {QStringLiteral("can_adapter"), QStringLiteral("PC 侧 CAN 适配器"), PreflightLevel::Ok,
            QStringLiteral("控制源=ros2，不经 CAN，跳过")};
  }
  return {QStringLiteral("can_adapter"), QStringLiteral("PC 侧 CAN 适配器"),
          device_present ? PreflightLevel::Ok : PreflightLevel::Warn,
          device_present
              ? QStringLiteral("CANalyst-II (04d8:0053) 已连接")
              : QStringLiteral("未检测到 CANalyst-II USB 设备（04d8:0053）；"
                               "control_source=%1 需要它读取 MCU 反馈")
                    .arg(control_source)};
}

// ===== 运行期探测（有系统调用，不在 gtest 里跑）=====

inline bool probe_ping(const QString& host, int timeout_s = 1) {
  QProcess proc;
  proc.start(QStringLiteral("ping"),
             {QStringLiteral("-c1"), QStringLiteral("-W"), QString::number(timeout_s), host});
  if (!proc.waitForFinished((timeout_s + 1) * 1000)) {
    proc.kill();
    return false;
  }
  return proc.exitCode() == 0;
}

// 读取 /sys/bus/usb/devices 下的 idVendor:idProduct，匹配 CANalyst-II
// （04d8:0053，见 CLAUDE.md「PC 的 USB CANalyst-II」）。只查存在性，不打开
// 设备——打开会和随后桥接进程的独占访问产生自我冲突（check_hil_ready.py
// 已知的假警报），体检阶段没必要冒这个险。
inline bool probe_can_adapter_present() {
  QDir devices_dir(QStringLiteral("/sys/bus/usb/devices"));
  const auto entries = devices_dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  for (const QString& entry : entries) {
    QFile vendor_file(devices_dir.filePath(entry) + QStringLiteral("/idVendor"));
    QFile product_file(devices_dir.filePath(entry) + QStringLiteral("/idProduct"));
    if (!vendor_file.open(QIODevice::ReadOnly) || !product_file.open(QIODevice::ReadOnly)) {
      continue;
    }
    const QString vendor = QString::fromLatin1(vendor_file.readAll()).trimmed();
    const QString product = QString::fromLatin1(product_file.readAll()).trimmed();
    if (vendor.compare(QStringLiteral("04d8"), Qt::CaseInsensitive) == 0 &&
        product.compare(QStringLiteral("0053"), Qt::CaseInsensitive) == 0) {
      return true;
    }
  }
  return false;
}

// 通过 /sys/class/drm/card*/device/vendor 的 PCI vendor ID 判断显卡厂商。
// 0x10de=NVIDIA，0x1002=AMD，0x8086=Intel。多卡时任意一张命中 NVIDIA 即报
// NVIDIA（体检只关心"有没有可能更稳的独显路径"，不做多卡精确建模）。
inline QString probe_gpu_vendor() {
  QDir drm_dir(QStringLiteral("/sys/class/drm"));
  const auto entries = drm_dir.entryList({QStringLiteral("card[0-9]*")}, QDir::Dirs);
  QString fallback;
  for (const QString& entry : entries) {
    QFile vendor_file(drm_dir.filePath(entry) + QStringLiteral("/device/vendor"));
    if (!vendor_file.open(QIODevice::ReadOnly)) continue;
    const QString vendor_id = QString::fromLatin1(vendor_file.readAll()).trimmed();
    if (vendor_id.compare(QStringLiteral("0x10de"), Qt::CaseInsensitive) == 0) {
      return QStringLiteral("NVIDIA");
    }
    if (vendor_id.compare(QStringLiteral("0x1002"), Qt::CaseInsensitive) == 0) {
      fallback = QStringLiteral("AMD");
    } else if (vendor_id.compare(QStringLiteral("0x8086"), Qt::CaseInsensitive) == 0 &&
               fallback.isEmpty()) {
      fallback = QStringLiteral("Intel");
    }
  }
  return fallback;
}

inline QVector<PreflightItem> run_preflight(const LaunchConfig& config) {
  QVector<PreflightItem> items;

  if (QString::fromLocal8Bit(qgetenv("ADAS_GUI_MODE"))
          .compare(QStringLiteral("sil"), Qt::CaseInsensitive) == 0) {
    // SIL 不依赖 CARLA、GPU、Orin、CAN 或 F280025C；启动器本身会在
    // ProcessManager 内定位 scripts/run_sil_fallback.sh，并把真实错误写入底部日志。
    items.push_back({QStringLiteral("sil_runtime"), QStringLiteral("SIL 运行环境"),
                     PreflightLevel::Ok,
                     QStringLiteral("本地闭环模式：跳过 CARLA / Orin / MCU / CAN 体检")});
    return items;
  }

  const QString carla_exe = carla_executable(config);
  items.push_back(classify_carla_executable(QFileInfo::exists(carla_exe), carla_exe));

  items.push_back(classify_gpu_quality(probe_gpu_vendor(), config.low_quality,
                                       config.render_offscreen));

  // Orin 连通性仅在「包含 Orin 远程启动」勾选时才做硬性阻断。
  // 默认 start_full_stack=false（PC 本地只跑 CARLA + bridge）时，Orin 不必
  // 通——比赛前一天操作员手动 systemctl start adas-hil.service 即可，GUI 不
  // 该因为 ping 不上 Orin 就把一次普通的 PC 启动拒掉。这是历史 bug：体检对
  // 所有启动路径都做 Orin 直连检查，导致默认配置下「按一键启动啥也不发生」。
  if (config.start_full_stack) {
    const QString peer = kDefaultOrinPeer();
    items.push_back(classify_orin_reachable(probe_ping(peer), peer));
  }

  const std::string iface = pick_direct_iface(enumerate_ipv4_ifaces());
  items.push_back(classify_direct_iface(QString::fromStdString(iface)));

  items.push_back(classify_can_adapter(config.control_source, probe_can_adapter_present()));

  return items;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__PREFLIGHT_CHECK_HPP_
