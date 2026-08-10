#ifndef ADAS_GUI__LAUNCH_CONFIG_HPP_
#define ADAS_GUI__LAUNCH_CONFIG_HPP_

// 一键启动的进程命令行构造。纯逻辑（仅 QString/QStringList），gtest 覆盖。
// 场景/控制源取值与 adas_carla_bridge/scenarios.py 的 ORDER、bridge_node.py
// 的 --control-source 保持一致；改动必须两侧同步。

#include <QString>
#include <QStringList>

namespace adas::gui {

struct LaunchConfig {
  QString carla_root;              // CARLA 安装目录（含 CarlaUE4.sh）
  QString scenario{"free"};        // 来自 known_scenarios() / scenarios.py ORDER
  QString town{"Town04"};
  QString control_source{"ros2"};  // ros2 / can / can_cpp
  QString can_transport{"canalystii"};
  QString can_interface{"can0"};
  int can_device_index{0};
  int can_channel{1};
  int can_bitrate{500000};
  double can_feedback_timeout_s{0.1};
  int carla_port{2000};
  bool low_quality{false};         // CARLA -quality-level=Low（远程/弱 GPU）
  bool render_offscreen{false};    // CARLA -RenderOffScreen（无本地窗口）
  // ---- Orin 远程栈 + MCU 烧录字段（默认 disable，比赛前手动起 Orin）----
  // 故意不放密码明文：密码由 SecureSettings 单例管，启动槽按 host+user
  // 取出来塞进 OrinStackManager，避免 LaunchConfig 被序列化时泄密。
  QString orin_host{"192.168.100.32"};
  QString orin_user{"jetson"};
  QString mcu_firmware_path{};     // 空 = 不烧录；非空 = 烧录后启栈
  bool start_full_stack{false};    // 默认只跑 PC 本地栈：比赛前一天操作员
                                    // 手动在 Orin 上 systemctl start
                                    // adas-hil.service 即可，不用 GUI 跨
                                    // 网 SSH。勾上后 GUI 才走 Orin 编排。
};

inline QStringList known_scenarios() {
  return {"lka", "acc", "acc_stop_and_go", "acc_slow_truck",
          "overtake", "aeb", "aeb_stationary", "aeb_pedestrian", "free"};
}

inline QStringList known_towns() {
  return {"Town01", "Town02", "Town03", "Town04", "Town05",
          "Town06", "Town07", "Town10HD"};
}

inline QStringList known_control_sources() { return {"ros2", "can", "can_cpp"}; }

inline QString carla_executable(const LaunchConfig& config) {
  return config.carla_root + "/CarlaUE4.sh";
}

inline QStringList carla_arguments(const LaunchConfig& config) {
  QStringList args;
  args << QString("-carla-rpc-port=%1").arg(config.carla_port);
  args << (config.low_quality ? "-quality-level=Low" : "-quality-level=Epic");
  if (config.render_offscreen) args << "-RenderOffScreen";
  return args;
}

// 桥接节点经 ros2 run 启动（GUI 自身在已 source 的环境里，子进程继承）。
inline QStringList bridge_arguments(const LaunchConfig& config) {
  QStringList args;
  args << "run" << "adas_carla_bridge" << "bridge_node"
       << "--scenario" << config.scenario
       << "--town" << config.town
       << "--carla-port" << QString::number(config.carla_port)
       // GUI“完整系统”由停止按钮管理生命周期，不能继承演示场景的
       // 60/90 秒默认时长后自行退出并让安全链进入 MRM。
       << "--duration" << "0"
       << "--control-source" << config.control_source;
   if (config.control_source == "can") {
     args << "--can-transport" << config.can_transport
          << "--can-device-index" << QString::number(config.can_device_index)
          << "--can-channel" << QString::number(config.can_channel)
          << "--can-bitrate" << QString::number(config.can_bitrate)
          << "--can-feedback-timeout-s"
          << QString::number(config.can_feedback_timeout_s, 'f', 3);
     if (config.can_transport == "socketcan") {
       args << "--can-interface" << config.can_interface;
     }
   }
   return args;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__LAUNCH_CONFIG_HPP_
