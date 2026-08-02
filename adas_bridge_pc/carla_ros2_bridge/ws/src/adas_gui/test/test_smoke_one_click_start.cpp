// 端到端烟雾测试：实例化真实的 ProcessManager，模拟「启动完整系统」按钮
// 的等价路径，验证：
//   (a) run_preflight 在 start_full_stack=false 时不强制 Orin ping；
//   (b) ProcessManager::startAll 能走到 startCarla，并在 CARLA 二进制
//       缺失时优雅报 Failed（而不是被 preflight 阻断到根本起不来）。
//
// 这是修复后的回归测试；修复前 (a) 会让 preflight 整体 Fail 把 startAll
// 截胡，整个 GUI 表现为"按一键启动啥也不发生"。

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QObject>
#include <QString>
#include <QTimer>

#include <iostream>
#include <sstream>

#include "preflight_check.hpp"
#include "process_manager.hpp"
#include "launch_config.hpp"

namespace adas::gui {
namespace {

class SmokeLogCapture : public QObject {
 public:
  explicit SmokeLogCapture(ProcessManager& mgr) {
    QObject::connect(&mgr, &ProcessManager::logLine, this,
                     [this](const QString& tag, const QString& line) {
                       std::lock_guard<std::mutex> g(mu_);
                       logs_.append(tag.toStdString() + " | " + line.toStdString() + "\n");
                     });
    QObject::connect(&mgr, &ProcessManager::carlaChanged, this,
                     [this](ProcState s, bool ready, const QString& detail) {
                       std::lock_guard<std::mutex> g(mu_);
                       std::ostringstream o;
                       o << "carla state=" << static_cast<int>(s)
                         << " ready=" << (ready ? "true" : "false")
                         << " detail=" << detail.toStdString() << "\n";
                       logs_.append(o.str());
                     });
    QObject::connect(&mgr, &ProcessManager::bridgeChanged, this,
                     [this](ProcState s, const QString& detail) {
                       std::lock_guard<std::mutex> g(mu_);
                       std::ostringstream o;
                       o << "bridge state=" << static_cast<int>(s)
                         << " detail=" << detail.toStdString() << "\n";
                       logs_.append(o.str());
                     });
  }
  std::string snapshot() const {
    std::lock_guard<std::mutex> g(mu_);
    return logs_;
  }
 private:
  mutable std::mutex mu_;
  std::string logs_;
};

TEST(SmokeOneClickStart, LocalStackPreflightSkipsOrinPing) {
  // 关键证据 1：start_full_stack=false 时，preflight 不应包含 Orin 检查。
  // 测试环境里 Orin 显然 ping 不通（除非 build host 与 HIL 网段重合）。
  // 如果包含 orin_ping 且 Orin ping 失败，整体 preflight 会把启动整体阻断。
  LaunchConfig cfg;
  cfg.carla_root = "/nonexistent/carla";
  cfg.scenario = "free";
  cfg.town = "Town04";
  cfg.control_source = "ros2";
  cfg.start_full_stack = false;

  const auto items = run_preflight(cfg);
  bool orin_check_present = false;
  QStringList seen_ids;
  for (const auto& it : items) {
    if (it.id == QStringLiteral("orin_ping")) orin_check_present = true;
    seen_ids.append(it.id);
  }
  EXPECT_FALSE(orin_check_present)
      << "本地栈不应触发 Orin ping，但 preflight 返回了 orin_ping 项；ids="
      << seen_ids.join(';').toStdString();
}

TEST(SmokeOneClickStart, LocalStackPreflightFailsOrinWhenFullStackRequested) {
  // 对照组：start_full_stack=true 时 Orin ping 必须出现。
  LaunchConfig cfg;
  cfg.carla_root = "/nonexistent/carla";
  cfg.start_full_stack = true;

  const auto items = run_preflight(cfg);
  bool orin_check_present = false;
  for (const auto& it : items) {
    if (it.id == QStringLiteral("orin_ping")) orin_check_present = true;
  }
  EXPECT_TRUE(orin_check_present)
      << "勾上 Orin 远程启动时，Orin ping 体检必须出现，否则没法阻断不可达的远端启动";
}

TEST(SmokeOneClickStart, StartAllReachesStartCarlaEvenWhenOrinUnreachable) {
  // 关键证据 2：startAll 链路在 CARLA 二进制缺失时走到 startCarla 并报 Failed，
  // 而不是被 preflight 截胡（修复前的历史行为）。
  ASSERT_TRUE(qApp != nullptr) << "QCoreApplication 未初始化";
  ProcessManager mgr;
  SmokeLogCapture capture(mgr);

  LaunchConfig cfg;
  cfg.carla_root = QStringLiteral("/nonexistent/carla");
  cfg.carla_port = 22222;  // 避开 2000（dev box 可能残留外部 CARLA 实例）
  cfg.scenario = QStringLiteral("free");
  cfg.town = QStringLiteral("Town04");
  cfg.control_source = QStringLiteral("ros2");
  cfg.start_full_stack = false;

  mgr.startAll(cfg);

  // 让信号在事件循环里走完
  QEventLoop loop;
  QTimer::singleShot(300, &loop, &QEventLoop::quit);
  loop.exec();

  const std::string log = capture.snapshot();
  // 1) startCarla 报 Failed，detail 含「未找到 CarlaUE4.sh」之类的错误。
  EXPECT_NE(log.find("未找到"), std::string::npos)
      << "startAll 应走到 startCarla 报 Failed，但日志里没看到 CARLA 二进制缺失的诊断：\n"
      << log;
  EXPECT_NE(log.find("CarlaUE4.sh"), std::string::npos)
      << "startAll 应明确指出缺少 CarlaUE4.sh：\n" << log;
  // 2) 因为 CARLA 没起来，桥不会被启；不应有 bridgeStarted 信号。
  EXPECT_EQ(mgr.bridgeState(), ProcState::Stopped);
  // 3) bridge_pending_ 应当被 CARLA 失败清掉（state change handler）。
  EXPECT_FALSE(mgr.hasManagedBridge());
}

}  // namespace
}  // namespace adas::gui

// gtest 的 main 由 ament_add_gtest 提供；这里需要 QCoreApplication 才能
// 让信号槽跑起来。在 gtest 的 main 之前用 gtest 的环境钩子建一个。
namespace {
struct QtApp {
  QtApp() {
    int argc = 1;
    static char app_name[] = "test_smoke_one_click_start";
    static char* argv[] = {app_name, nullptr};
    app_ = std::make_unique<QCoreApplication>(argc, argv);
  }
  std::unique_ptr<QCoreApplication> app_;
};
static QtApp g_qt_app;
}  // namespace