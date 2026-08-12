#ifndef ADAS_GUI__LAUNCH_CONFIG_HPP_
#define ADAS_GUI__LAUNCH_CONFIG_HPP_

// 一键启动的进程命令行构造。纯逻辑（仅 QString/QStringList），gtest 覆盖。
// 场景/控制源取值与 adas_carla_bridge/scenarios.py 的 ORDER、bridge_node.py
// 的 --control-source 保持一致；改动必须两侧同步。

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QProcessEnvironment>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>

#include <cmath>

#include "scenario_workflow.hpp"

namespace adas::gui {

struct LaunchConfig {
  // 后端是用户意图，不是从 ROS graph 猜出来的运行状态。
  QString backend{"carla_local_soc"};
  QString repo_root;               // 场景目录及编排脚本所在仓库根目录
  QString carla_root;              // CARLA 安装目录（含 CarlaUE4.sh）
  QString scenario{"free"};        // 来自 known_scenarios() / scenarios.py ORDER
  QString scenario_file;           // catalog.json 中的绝对场景文件路径
  int seed{0};
  int expected_actor_count{-1};
  QString acceptance_profile;
  double nav_distance_m{0.0};
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
  bool auto_navigation{true};      // 地图/位姿就绪后按场景 profile 自动生成目标
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
  // P0.C: GUI 编排本轮“启动场景”时生成 run_id，必须显式传给 carla_bridge。
  // 不传 → bridge 收到后会自己生成 UUID → 与 GUI/Planner 不一致 → 整个会话被拒。
  QString run_id;
};

struct ScenarioCatalogEntry {
  QString id;
  QString display_name;
  QString description;
  QString file;
  QString default_town{"Town04"};
  QStringList supported_backends;
  int default_seed{0};
  int expected_actor_count{-1};
  // P1.E: catalog schema v2 必填字段，GUI 必须显式消费，不能再依赖
  // scenario_workflow.hpp 的硬编码 fallback。
  double duration_s{0.0};
  QStringList actor_blueprints;
  QString acceptance_profile;
  double nav_distance_m{0.0};
};

inline QString find_repo_root(const QString& explicit_root = {}) {
  QStringList starts;
  if (!explicit_root.isEmpty()) starts << explicit_root;
  const auto env = QProcessEnvironment::systemEnvironment();
  starts << env.value(QStringLiteral("ADAS_REPO_ROOT"))
         << env.value(QStringLiteral("ADAS_SIL_ROOT"));
  if (QCoreApplication::instance()) starts << QCoreApplication::applicationDirPath();
  starts << QDir::currentPath();
  for (const QString& start : starts) {
    if (start.isEmpty()) continue;
    for (QDir dir(QFileInfo(start).isDir() ? start : QFileInfo(start).dir());;) {
      if (QFileInfo::exists(dir.filePath(QStringLiteral("scenarios/catalog.json")))) {
        return dir.absolutePath();
      }
      if (!dir.cdUp()) break;
    }
  }
  return {};
}

inline QVector<ScenarioCatalogEntry> load_scenario_catalog(
    const QString& explicit_root = {}) {
  QVector<ScenarioCatalogEntry> result;
  const QString root = find_repo_root(explicit_root);
  QFile file(QDir(root).filePath(QStringLiteral("scenarios/catalog.json")));
  if (root.isEmpty() || !file.open(QIODevice::ReadOnly)) return result;
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject() ||
      document.object().value(QStringLiteral("schema_version")).toInt() != 2) {
    return {};
  }
  const QJsonArray entries = document.object().value(QStringLiteral("scenarios")).toArray();
  if (entries.isEmpty()) return {};
  QSet<QString> ids;
  const QSet<QString> towns{"Town01", "Town02", "Town03", "Town04", "Town05",
                            "Town06", "Town07", "Town10HD"};
  const QSet<QString> backends{"carla", "sil"};
  const QStringList profile_ids = scenario_workflow_ids();
  const QSet<QString> profiles(profile_ids.cbegin(), profile_ids.cend());
  for (const QJsonValue& value : entries) {
    if (!value.isObject()) return {};
    const QJsonObject object = value.toObject();
    ScenarioCatalogEntry entry;
    entry.id = object.value(QStringLiteral("id")).toString();
    entry.display_name = object.value(QStringLiteral("display_name")).toString(entry.id);
    entry.description = object.value(QStringLiteral("description")).toString();
    entry.file = QDir(root).filePath(object.value(QStringLiteral("file")).toString());
    entry.default_town = object.value(QStringLiteral("default_town")).toString();
    entry.default_seed = object.value(QStringLiteral("default_seed")).toInt(0);
    entry.expected_actor_count =
        object.value(QStringLiteral("expected_actor_count")).toInt(-1);
    // P1.E: schema v2 必填字段。读不到或类型错误视作"catalog 缺关键数据"，
    // 直接丢弃这条 entry，避免 GUI 误用旧 profile 兜底。
    if (!object.contains(QStringLiteral("duration_s")) ||
        !object.value(QStringLiteral("duration_s")).isDouble()) {
      return {};
    }
    entry.duration_s = object.value(QStringLiteral("duration_s")).toDouble();
    const QJsonArray blueprints =
        object.value(QStringLiteral("actor_blueprints")).toArray();
    for (const QJsonValue& bp : blueprints) {
      if (bp.isNull()) {
        entry.actor_blueprints << QString();
      } else {
        entry.actor_blueprints << bp.toString();
      }
    }
    entry.acceptance_profile =
        object.value(QStringLiteral("acceptance_profile")).toString();
    if (!object.contains(QStringLiteral("nav_distance_m")) ||
        !object.value(QStringLiteral("nav_distance_m")).isDouble()) {
      return {};
    }
    entry.nav_distance_m = object.value(QStringLiteral("nav_distance_m")).toDouble();
    for (const QJsonValue& backend :
         object.value(QStringLiteral("supported_backends")).toArray()) {
      entry.supported_backends << backend.toString();
    }
    if (entry.id.isEmpty() || ids.contains(entry.id) || entry.display_name.isEmpty() ||
        entry.description.isEmpty() || !QFileInfo::exists(entry.file) ||
        !towns.contains(entry.default_town) || entry.supported_backends.isEmpty() ||
        entry.expected_actor_count < 0 ||
        entry.actor_blueprints.size() != entry.expected_actor_count ||
        !profiles.contains(entry.acceptance_profile) ||
        !std::isfinite(entry.nav_distance_m) || entry.nav_distance_m <= 0.0) {
      return {};
    }
    for (const QString& backend : entry.supported_backends) {
      if (!backends.contains(backend)) return {};
    }
    ids.insert(entry.id);
    result.push_back(entry);
  }
  return result;
}

inline QStringList legacy_scenarios() {
  // P1.E: 仅供离线单元测试验证"硬编码 ID 集合"——GUI 业务路径禁止调用。
  return {"lka", "acc", "acc_stop_and_go", "acc_slow_truck",
          "overtake", "aeb", "aeb_stationary", "aeb_pedestrian", "free",
          "dense_overtake_v1"};
}

inline QStringList known_scenarios(const QString& explicit_root = {}) {
  // P1.E: fail-closed。catalog 缺失或解析失败时**不**静默回退到 legacy；
  // 业务方必须把空列表视为 GUI 启动失败。
  const auto catalog = load_scenario_catalog(explicit_root);
  QStringList ids;
  for (const auto& entry : catalog) ids << entry.id;
  return ids;
}

inline ScenarioCatalogEntry scenario_catalog_entry(
    const QString& id, const QString& explicit_root = {}) {
  for (const auto& entry : load_scenario_catalog(explicit_root)) {
    if (entry.id == id) return entry;
  }
  ScenarioCatalogEntry fallback;
  fallback.id = id;
  return fallback;
}

inline QStringList known_backends() {
  // Only expose runnable operator profiles.  The unfinished HIL adapter stays
  // available to explicit developer tooling, but must not appear as a choice
  // that the GUI will deterministically reject.
  return {"carla_local_soc", "mil"};
}

inline QString backend_display_name(const QString& backend) {
  if (backend == QStringLiteral("mil")) return QStringLiteral("MIL 本机模拟硬件");
  if (backend == QStringLiteral("hil")) return QStringLiteral("HIL 实车硬件（待适配）");
  if (backend == QStringLiteral("carla_local_soc")) return QStringLiteral("CARLA 本机联合");
  if (backend == QStringLiteral("carla_hil") || backend == QStringLiteral("carla")) {
    return QStringLiteral("CARLA + Orin HIL");
  }
  if (backend == QStringLiteral("local_three_machine")) return QStringLiteral("本地三机测试");
  if (backend == QStringLiteral("sil_fallback")) return QStringLiteral("轻量 SIL 回退");
  return backend;
}

inline bool backend_uses_carla(const QString& backend) {
  return backend == QStringLiteral("carla_local_soc") ||
         backend == QStringLiteral("carla_hil") || backend == QStringLiteral("carla");
}

inline bool backend_uses_sil_domain(const QString& backend) {
  return backend == QStringLiteral("mil") ||
         backend == QStringLiteral("carla_local_soc") ||
         backend == QStringLiteral("local_three_machine") ||
         backend == QStringLiteral("sil_fallback");
}

inline QString catalog_backend(const QString& backend) {
  if (backend == QStringLiteral("mil")) return QStringLiteral("sil");
  if (backend == QStringLiteral("hil")) return QStringLiteral("carla");
  return backend_uses_carla(backend) ? QStringLiteral("carla") : QStringLiteral("sil");
}

inline bool scenario_supports_backend(const ScenarioCatalogEntry& scenario,
                                      const QString& backend) {
  return scenario.supported_backends.isEmpty() ||
         scenario.supported_backends.contains(catalog_backend(backend));
}

inline QString discover_carla_root() {
  const QString from_env = qEnvironmentVariable("CARLA_ROOT");
  // An explicit override is authoritative even when invalid; the preflight
  // must report that exact path instead of silently falling back.
  if (!from_env.isEmpty()) return from_env;
  const QString home = QDir::homePath();
  const QString preferred = QDir(home).filePath(QStringLiteral("程序/CARLA_0.9.16"));
  const QStringList candidates{preferred,
                               QDir(home).filePath(QStringLiteral("CARLA_0.9.16"))};
  for (const QString& candidate : candidates) {
    const QFileInfo executable(QDir(candidate).filePath(QStringLiteral("CarlaUE4.sh")));
    if (executable.isFile() && executable.isExecutable()) {
      return QDir(candidate).absolutePath();
    }
  }
  return preferred;
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
  args << "run" << "adas_carla_bridge" << "bridge_node";
  if (!config.scenario_file.isEmpty()) {
    args << "--scenario-file" << config.scenario_file
         << "--seed" << QString::number(config.seed);
    if (config.expected_actor_count >= 0) {
      args << "--expected-actor-count" << QString::number(config.expected_actor_count);
    }
  } else {
    args << "--scenario" << config.scenario;
  }
  // P0.C: 把 GUI 当前会话 run_id 透传给桥节点。bridge 现在 fail-closed,
  // 因此必须传入 RunIdSession::begin() 生成的规范 UUID v4。
  args << "--run-id" << config.run_id;
  args << "--town" << config.town
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

inline QStringList local_three_machine_arguments(const LaunchConfig& config) {
  QStringList args;
  if (!config.run_id.isEmpty()) args << "--run-id" << config.run_id;  // P0.C: 已固化,见 bridge_arguments
  if (!config.scenario_file.isEmpty()) {
    args << "--scenario-file" << config.scenario_file
         << "--seed" << QString::number(config.seed);
  } else {
    QString scenario = QStringLiteral("baseline");
    if (config.scenario.startsWith(QStringLiteral("aeb"))) scenario = "aeb";
    else if (config.scenario.startsWith(QStringLiteral("acc"))) scenario = "acc";
    else if (config.scenario.contains(QStringLiteral("overtake"))) scenario = "overtake";
    args << "--scenario" << scenario;
  }
  return args;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__LAUNCH_CONFIG_HPP_
