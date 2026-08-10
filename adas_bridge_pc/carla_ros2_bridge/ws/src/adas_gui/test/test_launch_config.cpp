// launch_config.hpp 进程命令行构造的单元测试。
// 取值契约：scenario ∈ scenarios.py ORDER，control-source ∈ bridge_node.py choices。
#include <gtest/gtest.h>

#include <QSettings>

#include "launch_config.hpp"

namespace {

using adas::gui::LaunchConfig;

TEST(LaunchConfig, CarlaExecutablePath) {
  LaunchConfig config;
  config.carla_root = "/opt/carla";
  EXPECT_EQ(adas::gui::carla_executable(config),
            QString("/opt/carla/CarlaUE4.sh"));
}

TEST(LaunchConfig, CarlaArgumentsDefaultEpic) {
  // LaunchConfig::render_offscreen 默认 false：比赛现场默认带本地渲染窗口
  // （PC 本地看 CARLA 视角），离屏渲染只在勾上「离屏渲染」时才启用。
  LaunchConfig config;
  const QStringList args = adas::gui::carla_arguments(config);
  EXPECT_TRUE(args.contains("-carla-rpc-port=2000"));
  EXPECT_TRUE(args.contains("-quality-level=Epic"));
  EXPECT_FALSE(args.contains("-RenderOffScreen"));
}

TEST(LaunchConfig, CarlaArgumentsWindowedModeOmitsOffscreenFlag) {
  LaunchConfig config;
  config.render_offscreen = false;
  const QStringList args = adas::gui::carla_arguments(config);
  EXPECT_FALSE(args.contains("-RenderOffScreen"));
}

TEST(LaunchConfig, CarlaArgumentsLowQualityOffscreen) {
  LaunchConfig config;
  config.low_quality = true;
  config.render_offscreen = true;
  config.carla_port = 2100;
  const QStringList args = adas::gui::carla_arguments(config);
  EXPECT_TRUE(args.contains("-carla-rpc-port=2100"));
  EXPECT_TRUE(args.contains("-quality-level=Low"));
  EXPECT_TRUE(args.contains("-RenderOffScreen"));
}

TEST(LaunchConfig, BridgeArgumentsRos2Source) {
  LaunchConfig config;
  config.scenario = "aeb";
  config.town = "Town03";
  const QStringList args = adas::gui::bridge_arguments(config);
  const QStringList expected{"run",        "adas_carla_bridge",
                             "bridge_node", "--scenario",
                             "aeb",         "--town",
                             "Town03",      "--carla-port",
                             "2000",        "--duration",
                             "0",           "--control-source",
                             "ros2"};
  EXPECT_EQ(args, expected);
}

TEST(LaunchConfig, BridgeArgumentsCanalystiiAddsProductionParameters) {
  LaunchConfig config;
  config.control_source = "can";
  const QStringList args = adas::gui::bridge_arguments(config);
  EXPECT_TRUE(args.contains("--can-transport"));
  EXPECT_TRUE(args.contains("canalystii"));
  EXPECT_TRUE(args.contains("--can-device-index"));
  EXPECT_TRUE(args.contains("--can-channel"));
  EXPECT_FALSE(args.contains("--can-interface"));
}

TEST(LaunchConfig, BridgeArgumentsSocketcanAddsInterface) {
  LaunchConfig config;
  config.control_source = "can";
  config.can_transport = "socketcan";
  config.can_interface = "can1";
  const QStringList args = adas::gui::bridge_arguments(config);
  const int index = args.indexOf("--can-interface");
  ASSERT_GE(index, 0);
  EXPECT_EQ(args.at(index + 1), QString("can1"));
}

TEST(LaunchConfig, KnownListsMatchBridgeContract) {
  // 与 scenarios.py ORDER / bridge_node.py --control-source 保持一致
  EXPECT_EQ(adas::gui::known_scenarios(),
            (QStringList{"lka", "acc", "acc_stop_and_go", "acc_slow_truck",
                         "overtake", "aeb", "aeb_stationary",
                         "aeb_pedestrian", "free"}));
  EXPECT_EQ(adas::gui::known_control_sources(),
            (QStringList{"ros2", "can", "can_cpp"}));
}

TEST(LaunchConfig, OrinHostUserDefaultsMatchKnownHarness) {
  LaunchConfig config;
  // 默认值对齐项目记忆：jetson@192.168.100.32（CAN: PEAK PCAN-USB → can1@500k）
  EXPECT_EQ(config.orin_host, QStringLiteral("192.168.100.32"));
  EXPECT_EQ(config.orin_user, QStringLiteral("jetson"));
  EXPECT_EQ(config.can_bitrate, 500000);
  EXPECT_EQ(config.carla_port, 2000);
}

TEST(LaunchConfig, StartFullStackDefaultIsFalse) {
  // 比赛当天稳为主：默认只跑 PC 本地栈（CARLA + bridge）。Orin 端由
  // 操作员在比赛前手动 systemctl start adas-hil.service 即可。SSH 编排
  // 是高阶能力（比赛当天调试通了再勾选「包含 Orin 远程启动」复选框）。
  LaunchConfig config;
  EXPECT_FALSE(config.start_full_stack)
      << "默认必须 false，避免 GUI 自动 ssh 到 Orin 在比赛现场出岔子";
}

TEST(LaunchConfig, LaunchConfigDoesNotCarryPlaintextPassword) {
  // 设计契约：LaunchConfig 不持密码字段；密码走 SecureSettings 单例。
  // 这是显式断言，不是字段遗漏——若有人后续加了 orin_password 字段，
  // 此断言提醒重新设计 SecureSettings 拆分。
  LaunchConfig config;
  EXPECT_TRUE(config.mcu_firmware_path.isEmpty())
      << "mcu_firmware_path 默认应为空（首次启动不烧录）";
}

}  // namespace
