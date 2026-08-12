// 后端边界烟雾测试：MIL 只依赖本机模拟硬件环境；HIL 只保留接口，且不能
// 在尚未适配时触发任何远程或硬件动作。

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

TEST(SmokeOneClickStart, MilPreflightOnlyChecksLocalRuntime) {
  const QByteArray previous = qgetenv("ROS_DOMAIN_ID");
  qputenv("ROS_DOMAIN_ID", "145");
  LaunchConfig cfg;
  cfg.backend = QStringLiteral("mil");

  const auto items = run_preflight(cfg);
  QStringList seen_ids;
  for (const auto& it : items) seen_ids.append(it.id);
  EXPECT_TRUE(seen_ids.contains(QStringLiteral("dds_profile")));
  EXPECT_TRUE(seen_ids.contains(QStringLiteral("sil_runtime")));
  EXPECT_FALSE(seen_ids.contains(QStringLiteral("carla_exe")));
  EXPECT_FALSE(seen_ids.contains(QStringLiteral("orin_ping")));
  if (previous.isNull()) qunsetenv("ROS_DOMAIN_ID");
  else qputenv("ROS_DOMAIN_ID", previous);
}

TEST(SmokeOneClickStart, HilPreflightIsExplicitlyBlocked) {
  const QByteArray previous = qgetenv("ROS_DOMAIN_ID");
  qputenv("ROS_DOMAIN_ID", "43");
  LaunchConfig cfg;
  cfg.backend = QStringLiteral("hil");

  const auto items = run_preflight(cfg);
  bool hil_blocked = false;
  for (const auto& it : items) {
    if (it.id == QStringLiteral("hil_stage") && it.level == PreflightLevel::Fail) {
      hil_blocked = true;
    }
  }
  EXPECT_TRUE(hil_blocked);
  if (previous.isNull()) qunsetenv("ROS_DOMAIN_ID");
  else qputenv("ROS_DOMAIN_ID", previous);
}

TEST(SmokeOneClickStart, HilStartAllPerformsNoHardwareAction) {
  ASSERT_TRUE(qApp != nullptr) << "QCoreApplication 未初始化";
  ProcessManager mgr;
  SmokeLogCapture capture(mgr);

  LaunchConfig cfg;
  cfg.backend = QStringLiteral("hil");
  cfg.run_id = QStringLiteral("11111111-2222-4333-8444-555555555555");

  mgr.startAll(cfg);

  const std::string log = capture.snapshot();
  EXPECT_NE(log.find("HIL 后端尚未适配"), std::string::npos) << log;
  EXPECT_EQ(mgr.bridgeState(), ProcState::Stopped);
  EXPECT_FALSE(mgr.hasManagedProcesses());
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
