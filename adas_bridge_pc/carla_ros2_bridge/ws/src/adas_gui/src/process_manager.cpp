#include "process_manager.hpp"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcessEnvironment>
#include <QTimer>

#include <csignal>
#include <unistd.h>

namespace adas::gui {

namespace {
constexpr int kCarlaReadinessTimeoutSeconds = 180;
}

QString proc_state_name(ProcState state) {
  switch (state) {
    case ProcState::Stopped: return QStringLiteral("未运行");
    case ProcState::Starting: return QStringLiteral("启动中");
    case ProcState::Running: return QStringLiteral("运行中");
    case ProcState::Stopping: return QStringLiteral("停止中");
    case ProcState::Failed: return QStringLiteral("异常退出");
  }
  return QStringLiteral("?");
}

ManagedProcess::ManagedProcess(QString tag, QObject* parent)
    : QObject(parent), tag_(std::move(tag)) {
  // 子进程放入独立进程组，stop() 对整组发信号（CarlaUE4.sh → UE4 子进程）。
  process_.setChildProcessModifier([]() { ::setpgid(0, 0); });
  // stdout/stderr 分离通道：stdout 走 tag（如 "CARLA"/"桥接"），
  // stderr 走 "ERROR" tag，自动升级为故障事件。
  process_.setProcessChannelMode(QProcess::SeparateChannels);

  connect(&process_, &QProcess::readyReadStandardOutput, this, [this]() {
    forward_output(/*stderr=*/false);
  });
  connect(&process_, &QProcess::readyReadStandardError, this, [this]() {
    forward_output(/*stderr=*/true);
  });
  connect(&process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError) {
    if (state_ == ProcState::Starting) {
      set_state(ProcState::Failed, process_.errorString());
    }
  });
  connect(&process_, &QProcess::started, this, [this]() {
    started_at_ms_ = QDateTime::currentMSecsSinceEpoch();
    set_state(ProcState::Running);
  });
  connect(&process_,
          QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this](int code, QProcess::ExitStatus exit_status) {
            kill_timer_.stop();
            // 退出时把缓冲的 stdout/stderr 全部 flush，避免尾行丢失
            forward_output(/*stderr=*/false);
            forward_output(/*stderr=*/true);
            const bool expected = state_ == ProcState::Stopping;
            const bool clean =
                exit_status == QProcess::NormalExit && code == 0;
            set_state(expected || clean ? ProcState::Stopped : ProcState::Failed,
                      QStringLiteral("exit=%1").arg(code));
          });

  kill_timer_.setSingleShot(true);
  kill_timer_.setInterval(5000);
  connect(&kill_timer_, &QTimer::timeout, this, [this]() {
    // pid<=0 时 kill(-pid) 会打到自身进程组，必须先判
    const pid_t pid = static_cast<pid_t>(process_.processId());
    if (process_.state() != QProcess::NotRunning && pid > 0) {
      ::kill(-pid, SIGKILL);
    }
  });
}

void ManagedProcess::start(const QString& program, const QStringList& arguments) {
  if (process_.state() != QProcess::NotRunning) return;
  line_buffer_.clear();
  emit logLine(tag_, QStringLiteral("$ %1 %2").arg(program, arguments.join(' ')));
  set_state(ProcState::Starting);
  process_.start(program, arguments);
}

void ManagedProcess::stop() {
  if (process_.state() == QProcess::NotRunning) return;
  set_state(ProcState::Stopping);
  const pid_t pid = static_cast<pid_t>(process_.processId());
  if (pid > 0) {
    ::kill(-pid, SIGTERM);   // 整个进程组（CarlaUE4.sh + UE4 子进程）
    kill_timer_.start();
  } else {
    process_.kill();         // 尚未真正 spawn，直接终止
  }
}

void ManagedProcess::set_state(ProcState state, const QString& detail) {
  if (state_ == state && detail.isEmpty()) return;
  state_ = state;
  emit stateChanged(state, detail);
}

void ManagedProcess::forward_output(bool stderr) {
  // stderr=true 用 "ERROR" tag 让 LogDrawer 自动升级为故障事件；
  // stdout 用子进程的 tag（如 "CARLA"/"桥接"）。
  const QString tag = stderr ? QStringLiteral("ERROR") : tag_;
  QByteArray chunk = stderr ? process_.readAllStandardError()
                            : process_.readAllStandardOutput();
  line_buffer_ += chunk;
  int newline;
  while ((newline = line_buffer_.indexOf('\n')) >= 0) {
    const QString line = QString::fromUtf8(line_buffer_.left(newline)).trimmed();
    line_buffer_.remove(0, newline + 1);
    if (!line.isEmpty()) emit logLine(tag, line);
  }
}

ProcessManager::ProcessManager(QObject* parent)
    : QObject(parent), carla_(QStringLiteral("CARLA")),
      bridge_(QStringLiteral("桥接")) {
  sil_mode_ = QString::fromLocal8Bit(qgetenv("ADAS_GUI_MODE"))
                  .compare(QStringLiteral("sil"), Qt::CaseInsensitive) == 0;
  connect(&carla_, &ManagedProcess::logLine, this, &ProcessManager::logLine);
  connect(&bridge_, &ManagedProcess::logLine, this, &ProcessManager::logLine);
  // Orin 远端命令的 stdout/stderr 走同一 LogDrawer（tag 由 OrinStackManager 自带）
  connect(&orin_, &OrinStackManager::logLine, this, &ProcessManager::logLine);
  // Orin 命令完成 → 推进全流程状态机
  connect(&orin_, &OrinStackManager::finished, this,
          &ProcessManager::on_orin_command_finished);
  // 绿灯轮询 timer
  green_light_timer_.setInterval(500);
  connect(&green_light_timer_, &QTimer::timeout, this,
          &ProcessManager::check_green_light);
  // Phase 2 hardening：烧录超时 timer — 若 dslite 180s 内未结束,强制 SIGKILL。
  connect(&flash_timeout_timer_, &QTimer::timeout, this, [this]() {
    if (!orin_.busy()) return;
    emit logLine(QStringLiteral("MCU"),
                 QStringLiteral("! 烧录超时（180s）：SIGKILL dslite 进程组"));
    // 注意：dslite 原子性会被破坏,MCU 可能 brick。GUI 必须显式告诉用户。
    emit flashFinished(false,
                       QStringLiteral("烧录 180s 超时,已 SIGKILL dslite;"
                                      "若 MCU 仍未响应需手动硬件复位"));
    orin_.kill();  // 强制 kill 进程组
  });
  connect(&bridge_, &ManagedProcess::stateChanged, this,
          [this](ProcState state, const QString& detail) {
            if (state != ProcState::Stopped) external_bridge_ = false;
            if (state == ProcState::Running) {
              emit logLine(QStringLiteral("STARTUP"),
                           QStringLiteral("[STARTUP][BRIDGE_PROCESS_STARTED]"));
            }
            // 全流程编排：如果桥在 StartBridge 阶段 Failed，主动失败
            on_bridge_state_changed_for_topology(state, detail);
            emit bridgeChanged(state, detail);
          });
  connect(&carla_, &ManagedProcess::stateChanged, this,
          [this](ProcState state, const QString& detail) {
            if (state == ProcState::Running) {
              external_carla_ = false;
              emit logLine(
                  QStringLiteral("STARTUP"),
                  QStringLiteral("[STARTUP][CARLA_PROCESS_STARTED] rpc_host=127.0.0.1 "
                                 "rpc_port=%1")
                      .arg(pending_config_.carla_port));
              start_readiness_probe();
            } else {
              stop_readiness_probe();
              carla_ready_ = false;
              if (state == ProcState::Stopped || state == ProcState::Failed) {
                bridge_pending_ = false;
                // 只有本 GUI 真正 start() 过的 CARLA 才持有这把锁（探测到外部
                // 实例复用时从不加锁）；这里无条件 unlock 是安全的空操作。
                carla_lock_.unlock();
              }
            }
            // 全流程编排：如果 CARLA 进程在 StartCarla 阶段 Failed，主动失败
            on_carla_state_changed_for_topology(state, carla_ready_, detail);
            emit carlaChanged(state, carla_ready_, detail);
          });

  connect(&readiness_probe_,
          QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          &ProcessManager::on_readiness_finished);
  connect(&readiness_probe_, &QProcess::started, this, [this]() {
    emit logLine(
        QStringLiteral("STARTUP"),
        QStringLiteral("[STARTUP][CARLA_READINESS_STARTED] pid=%1 python=%2 "
                       "script=%3 timeout_ms=%4 generation=%5")
            .arg(readiness_probe_.processId())
            .arg(readiness_probe_.program())
            .arg(readiness_probe_.arguments().value(0))
            .arg((kCarlaReadinessTimeoutSeconds + 5) * 1000)
            .arg(readiness_generation_));
  });
  connect(&readiness_probe_, &QProcess::readyReadStandardOutput, this, [this]() {
    const QByteArray chunk = readiness_probe_.readAllStandardOutput();
    readiness_stdout_ += chunk;
    emit logLine(QStringLiteral("STARTUP"),
                 QStringLiteral("[STARTUP][CARLA_READINESS_STDOUT] %1")
                     .arg(QString::fromUtf8(chunk).trimmed()));
  });
  connect(&readiness_probe_, &QProcess::readyReadStandardError, this, [this]() {
    const QByteArray chunk = readiness_probe_.readAllStandardError();
    readiness_stderr_ += chunk;
    emit logLine(QStringLiteral("STARTUP"),
                 QStringLiteral("[STARTUP][CARLA_READINESS_STDERR] %1")
                     .arg(QString::fromUtf8(chunk).trimmed()));
  });
  connect(&readiness_probe_, &QProcess::errorOccurred, this,
          [this](QProcess::ProcessError) {
            if (!readiness_running_) return;
            readiness_running_ = false;
            readiness_timeout_timer_.stop();
            carla_ready_ = false;
            const QString detail =
                QStringLiteral("readiness 子进程异常（%1）：%2")
                    .arg(readiness_probe_.program(), readiness_probe_.errorString());
            emit carlaChanged(carla_.state(), false, detail);
            if (full_stage_ == FullStage::WaitReadiness) {
              fail_full_stack(QStringLiteral("readiness"), detail);
            }
          });
  readiness_timeout_timer_.setSingleShot(true);
  connect(&readiness_timeout_timer_, &QTimer::timeout, this, [this]() {
    if (!readiness_running_) return;
    emit logLine(QStringLiteral("STARTUP"),
                 QStringLiteral("[STARTUP][CARLA_READINESS_TIMEOUT] generation=%1")
                     .arg(readiness_generation_));
    readiness_probe_.kill();
  });

  hil_manager_.setChildProcessModifier([]() { ::setpgid(0, 0); });
  hil_manager_.setProcessChannelMode(QProcess::SeparateChannels);
  connect(&hil_manager_, &QProcess::readyReadStandardOutput, this, [this]() {
    forward_hil_output(hil_manager_, hil_output_buffer_, false);
  });
  connect(&hil_manager_, &QProcess::readyReadStandardError, this, [this]() {
    forward_hil_output(hil_manager_, hil_error_buffer_, true);
  });
  connect(&hil_manager_, &QProcess::started, this, [this]() {
    emit carlaChanged(ProcState::Starting, false,
                      sil_mode_ ? QStringLiteral("统一 SIL 管理器已启动")
                                : QStringLiteral("统一 HIL 管理器已启动"));
    emit bridgeChanged(ProcState::Starting,
                       sil_mode_ ? QStringLiteral("等待 SIL 话题就绪")
                                 : QStringLiteral("等待 CARLA 就绪后自动启动"));
  });
  connect(&hil_manager_, &QProcess::errorOccurred, this,
          [this](QProcess::ProcessError) {
            const QString detail =
                QStringLiteral("无法启动统一 HIL 管理器：%1").arg(hil_manager_.errorString());
            emit carlaChanged(ProcState::Failed, false, detail);
            emit bridgeChanged(ProcState::Failed, detail);
          });
  connect(&hil_manager_,
          QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this](int code, QProcess::ExitStatus status) {
            forward_hil_output(hil_manager_, hil_output_buffer_, false);
            forward_hil_output(hil_manager_, hil_error_buffer_, true);
            const bool expected = hil_stop_requested_;
            hil_stop_requested_ = false;
            carla_ready_ = false;
            external_carla_ = false;
            external_bridge_ = false;
            const bool clean = status == QProcess::NormalExit && (code == 0 || expected);
            const ProcState state = clean ? ProcState::Stopped : ProcState::Failed;
            const QString detail = expected ? QStringLiteral("完整系统已停止")
                                            : (sil_mode_ ? QStringLiteral("SIL 管理器退出：exit=%1")
                                                         : QStringLiteral("统一管理器退出：exit=%1"))
                                                .arg(code);
            emit carlaChanged(state, false, detail);
            emit bridgeChanged(state, detail);
          });

  stop_helper_.setProcessChannelMode(QProcess::SeparateChannels);
  connect(&stop_helper_, &QProcess::readyReadStandardOutput, this, [this]() {
    forward_hil_output(stop_helper_, stop_output_buffer_, false);
  });
  connect(&stop_helper_, &QProcess::readyReadStandardError, this, [this]() {
    forward_hil_output(stop_helper_, stop_error_buffer_, true);
  });
  connect(&stop_helper_, &QProcess::errorOccurred, this,
          [this](QProcess::ProcessError) {
            const QString detail =
                QStringLiteral("无法启动停止工具：%1").arg(stop_helper_.errorString());
            emit carlaChanged(ProcState::Failed, false, detail);
            emit bridgeChanged(ProcState::Failed, detail);
          });
  connect(&stop_helper_,
          QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this](int code, QProcess::ExitStatus status) {
            forward_hil_output(stop_helper_, stop_output_buffer_, false);
            forward_hil_output(stop_helper_, stop_error_buffer_, true);
            const bool clean = status == QProcess::NormalExit && code == 0;
            emit carlaChanged(clean ? ProcState::Stopped : ProcState::Failed, false,
                              clean ? QStringLiteral("完整系统已停止")
                                    : QStringLiteral("停止失败：exit=%1").arg(code));
            emit bridgeChanged(clean ? ProcState::Stopped : ProcState::Failed,
                               clean ? QStringLiteral("完整系统已停止")
                                     : QStringLiteral("停止失败：exit=%1").arg(code));
          });
}

ProcessManager::~ProcessManager() {
  stop_readiness_probe();
  bridge_.stop();
  carla_.stop();
  const int kChildKillWaitMs = 5000;
  if (bridge_.active()) {
    if (!bridge_.waitForFinished(kChildKillWaitMs)) bridge_.kill();
  }
  if (carla_.active()) {
    if (!carla_.waitForFinished(kChildKillWaitMs)) carla_.kill();
  }
  if (hil_manager_.state() != QProcess::NotRunning) {
    hil_stop_requested_ = true;
    hil_manager_.terminate();
    if (!hil_manager_.waitForFinished(30000)) {
      hil_manager_.kill();
      hil_manager_.waitForFinished(5000);
    }
  }
  if (stop_helper_.state() != QProcess::NotRunning &&
      !stop_helper_.waitForFinished(30000)) {
    stop_helper_.kill();
    stop_helper_.waitForFinished(5000);
  }
}

namespace {

// 点击时刻的同步探测：识别外部已启动的 CARLA，避免二次拉起抢端口。这只是
// "要不要再拉起一个 CARLA 进程"的冲突检测，不是就绪判据——即使端口已开，
// 后续仍必须过 start_readiness_probe() 才允许启桥。
bool port_open_now(int port) {
  QTcpSocket socket;
  socket.connectToHost(QStringLiteral("127.0.0.1"), static_cast<quint16>(port));
  const bool open = socket.waitForConnected(200);
  socket.abort();
  return open;
}

// 定位 scripts/carla_readiness.py。该脚本住在 adas_pc/scripts/，在 colcon
// 安装树（ws/install/...）之外，ament index 找不到；改为从可执行文件目录向上
// 逐级搜索 "scripts/carla_readiness.py"，兼容 source 内直接运行与安装后运行
// 两种布局。ADAS_PC_ROOT 环境变量可显式覆盖（非常规部署/测试注入用）。
QString find_carla_readiness_script() {
  const QString override_root =
      QProcessEnvironment::systemEnvironment().value(QStringLiteral("ADAS_PC_ROOT"));
  if (!override_root.isEmpty()) {
    const QString candidate =
        QDir(override_root).filePath(QStringLiteral("scripts/carla_readiness.py"));
    if (QFileInfo::exists(candidate)) return candidate;
  }
  QDir dir(QCoreApplication::applicationDirPath());
  for (int up = 0; up < 12; ++up) {
    const QString candidate = dir.filePath(QStringLiteral("scripts/carla_readiness.py"));
    if (QFileInfo::exists(candidate)) return candidate;
    if (!dir.cdUp()) break;
  }
  return QString();
}

QString find_release_script(const QString& name) {
  QDir dir(QCoreApplication::applicationDirPath());
  for (int up = 0; up < 12; ++up) {
    const QString candidate =
        dir.filePath(QStringLiteral("scripts/release/%1").arg(name));
    if (QFileInfo::exists(candidate)) return candidate;
    if (!dir.cdUp()) break;
  }
  return QString();
}

QString find_sil_script() {
  const QString override_root =
      QProcessEnvironment::systemEnvironment().value(QStringLiteral("ADAS_SIL_ROOT"));
  if (!override_root.isEmpty()) {
    const QString candidate =
        QDir(override_root).filePath(QStringLiteral("scripts/run_sil_fallback.sh"));
    if (QFileInfo::exists(candidate)) return candidate;
  }

  // 安装后的 adas_gui 位于 ws/install/adas_gui/lib/adas_gui；向上搜索即可
  // 找到 bowen_ADAS/scripts/run_sil_fallback.sh。也支持从源码目录启动。
  for (QDir dir(QCoreApplication::applicationDirPath());; ) {
    const QString candidate = dir.filePath(QStringLiteral("scripts/run_sil_fallback.sh"));
    if (QFileInfo::exists(candidate)) return candidate;
    if (!dir.cdUp()) break;
  }
  for (QDir dir(QDir::currentPath());; ) {
    const QString candidate = dir.filePath(QStringLiteral("scripts/run_sil_fallback.sh"));
    if (QFileInfo::exists(candidate)) return candidate;
    if (!dir.cdUp()) break;
  }
  return QString();
}

QString sil_scenario_name(const QString& scenario) {
  if (scenario.startsWith(QStringLiteral("aeb"), Qt::CaseInsensitive)) {
    return QStringLiteral("aeb");
  }
  if (scenario.startsWith(QStringLiteral("acc"), Qt::CaseInsensitive)) {
    return QStringLiteral("acc");
  }
  if (scenario.contains(QStringLiteral("overtake"), Qt::CaseInsensitive)) {
    return QStringLiteral("overtake");
  }
  return QStringLiteral("baseline");
}

}  // namespace

void ProcessManager::startCarla(const LaunchConfig& config) {
  if (sil_mode_) {
    start_sil_stack(config);
    return;
  }
  pending_config_ = config;
  if (port_open_now(config.carla_port)) {
    external_carla_ = true;
    // 端口已开只说明"不用再拉起一个 CARLA 进程"，不等于"可以启桥"——外部
    // 实例同样必须过 readiness 脚本（RPC/版本/世界/稳定窗口），否则一个刚
    // 被别的脚本拉起、还没稳定的外部 CARLA 会被当场判定就绪。
    emit carlaChanged(carla_.state(), false,
                      QStringLiteral("检测到外部 CARLA（端口 %1），校验就绪中…")
                          .arg(config.carla_port));
    start_readiness_probe();
    return;
  }
  const QString executable = carla_executable(config);
  if (!QFileInfo::exists(executable)) {
    emit carlaChanged(ProcState::Failed, false,
                      QStringLiteral("未找到 %1（请设置 CARLA 目录）")
                          .arg(executable));
    return;
  }
  // 系统级锁：端口探测只能防住"本机已经在监听"，防不住两个客户端同一
  // 瞬间都探测到端口空闲、都决定拉起的race（CarlaUE4 从进程起来到端口就绪
  // 有数秒空档）。flock 非阻塞，抢不到锁直接拒绝，不重试、不排队。
  if (!carla_lock_.tryLock()) {
    external_carla_ = true;
    emit carlaChanged(ProcState::Failed, false,
                      QStringLiteral("另一个 CARLA 实例正在启动/运行中（系统锁 %1 已被占用），"
                                     "本机同一时刻只允许一个 CARLA")
                          .arg(carla_lock_path()));
    return;
  }
  external_carla_ = false;
  carla_ready_ = false;
  emit logLine(
      QStringLiteral("STARTUP"),
      QStringLiteral("[STARTUP][CARLA_START_REQUESTED] command=%1 %2 "
                     "rpc_host=127.0.0.1 rpc_port=%3")
          .arg(executable, carla_arguments(config).join(' '))
          .arg(config.carla_port));
  carla_.start(executable, carla_arguments(config));
}

void ProcessManager::startBridge(const LaunchConfig& config) {
  if (sil_mode_) {
    if (bridge_probe_ && bridge_probe_()) {
      external_carla_ = true;
      external_bridge_ = true;
      carla_ready_ = true;
      emit carlaChanged(ProcState::Running, true, QStringLiteral("外部 SIL · 话题已就绪"));
      emit bridgeChanged(ProcState::Running, QStringLiteral("外部 SIL · ROS graph 已发现"));
      return;
    }
    start_sil_stack(config);
    return;
  }
  const bool graph_has_bridge = bridge_probe_ ? bridge_probe_() : external_bridge_;
  external_bridge_ = graph_has_bridge;
  emit logLine(QStringLiteral("桥接"),
               QStringLiteral("startBridge: graph_has_bridge=%1 external_bridge_=%2")
                   .arg(graph_has_bridge).arg(external_bridge_));
  if (graph_has_bridge) {
    external_bridge_ = true;
    emit logLine(QStringLiteral("桥接"),
                 QStringLiteral("检测到外部 carla_bridge，复用现有实例，跳过重复启动"));
    emit bridgeChanged(ProcState::Running, QStringLiteral("外部实例 · ROS graph 已发现"));
    return;
  }
  // Orin is ROS 2 Humble while this PC is Jazzy. Direct cross-distro DDS
  // discovery corrupts ros_discovery_info and prevents the control stack from
  // consuming bridge data. Run the ROS-facing bridge in the pinned Humble
  // container; CARLA remains a native host process reachable via host network.
  const QString readiness_script = find_carla_readiness_script();
  const QString bridge_runner =
      readiness_script.isEmpty()
          ? QString()
          : QFileInfo(readiness_script).dir().filePath(
                QStringLiteral("run_humble_bridge.sh"));
  if (bridge_runner.isEmpty() || !QFileInfo::exists(bridge_runner)) {
    emit bridgeChanged(
        ProcState::Failed,
        QStringLiteral("缺少 Humble Bridge 启动器：adas_pc/scripts/run_humble_bridge.sh"));
    return;
  }
  const QStringList args = bridge_arguments(config).mid(3);
  emit logLine(QStringLiteral("STARTUP"),
               QStringLiteral("[STARTUP][BRIDGE_START_REQUESTED] command=%1 %2 "
                              "ros_runtime=humble-container")
                   .arg(bridge_runner, args.join(' ')));
  bridge_.start(bridge_runner, args);
}

void ProcessManager::setExternalCarlaDetected(bool detected) {
  if (carla_.active()) return;
  if (external_carla_ == detected && carla_ready_ == detected) return;
  external_carla_ = detected;
  carla_ready_ = detected;
  emit carlaChanged(carlaState(), detected,
                    detected ? QStringLiteral("统一管理器实例 · 数据已就绪")
                             : QStringLiteral("统一管理器实例已离线"));
}

void ProcessManager::setExternalBridgeDetected(bool detected) {
  // 自己启动的进程由 QProcess 生命周期负责，不能被 graph 短暂抖动覆盖。
  if (bridge_.state() != ProcState::Stopped) return;
  if (external_bridge_ == detected) return;
  external_bridge_ = detected;
  emit bridgeChanged(detected ? ProcState::Running : ProcState::Stopped,
                     detected ? QStringLiteral("外部实例 · ROS graph 已发现")
                              : QStringLiteral("外部实例已离线"));
}

void ProcessManager::startAll(const LaunchConfig& config) {
  if (sil_mode_) {
    ++startup_generation_;
    if (bridge_probe_ && bridge_probe_()) {
      external_carla_ = true;
      external_bridge_ = true;
      carla_ready_ = true;
      emit logLine(QStringLiteral("SIL"),
                   QStringLiteral("检测到已运行 SIL，复用现有闭环，不重复启动"));
      emit carlaChanged(ProcState::Running, true, QStringLiteral("外部 SIL · 话题已就绪"));
      emit bridgeChanged(ProcState::Running, QStringLiteral("外部 SIL · ROS graph 已发现"));
      emit stackProgress(QStringLiteral("complete"), QStringLiteral("SIL 闭环已连接 ✅"));
    } else {
      start_sil_stack(config);
    }
    return;
  }
  if (readiness_running_) {
    bridge_pending_ = true;
    emit logLine(QStringLiteral("STARTUP"),
                 QStringLiteral("[STARTUP][START_REQUEST_COALESCED] generation=%1")
                     .arg(startup_generation_));
    return;
  }
  ++startup_generation_;
  const QString release_script = find_release_script(QStringLiteral("start_hil.sh"));
  if (!release_script.isEmpty()) {
    start_release_stack(config, release_script);
    return;
  }
  // 新增：start_full_stack=true → 走 Orin+CARLA+bridge 拓扑
  if (config.start_full_stack) {
    start_full_stack(config);
    return;
  }
  // 本地栈（CARLA + bridge）：永远把 bridge_pending_ 置位，让 CARLA 就绪后
  // 自动接桥；这条链只受 startBridge 内的 ROS graph 探针（避免双实例）阻拦，
  // 不会被任何 CARLA 进程状态截断。
  pending_config_ = config;
  bridge_pending_ = true;
  if (carla_ready_) {
    bridge_pending_ = false;
    startBridge(config);
    return;
  }
  if (carla_.state() == ProcState::Stopped || carla_.state() == ProcState::Failed) {
    // readiness 校验总是异步的（含外部 CARLA 复用），startCarla() 内部会拉起
    // carla_readiness.py；桥的实际启动统一在 on_readiness_finished() 里，
    // 靠 bridge_pending_ 接续，这里不再假设 carla_ready_ 会同步置位。
    startCarla(config);
  } else if (carla_.state() == ProcState::Running) {
    // CARLA 已跑但 readiness 还没过（冷启动首次超时 / 上次失败）：
    // 重试一次 probe。否则再点「启动完整系统」是死按钮——
    // CARLA 不会被重新拉起、也没有新校验发生。
    if (!readiness_running_) start_readiness_probe();
  } else if (carla_.state() == ProcState::Starting) {
    // CARLA 进程正在拉起中，stateChanged→Running 会自动接 start_readiness_probe，
    // 桥启动经由 on_readiness_finished 接续；这里只显式打个日志，避免按钮
    // 在 Starting 窗口内被点多次时表现为"啥也没发生"。
    emit logLine(QStringLiteral("启动"),
                 QStringLiteral("CARLA 启动中，将在 readiness 校验通过后自动启桥"));
  } else {
    // Stopping：用户当前正在收尾，刻意不动。
  }
}

void ProcessManager::stopAll() {
  ++startup_generation_;
  bridge_pending_ = false;
  stop_readiness_probe();
  // Orin HIL 是系统级常驻服务。GUI 的“停止完整系统”只停止本机
  // Bridge/CARLA，绝不停止 Orin，也不撤销 MCU 会话；keeper timer 会保证
  // adas-hil 即使被外部 stop 也自动恢复。
  if (full_stack_running()) {
    full_stage_ = FullStage::Failed;  // 阻止后续 on_orin_command_finished 误推进
    full_orin_password_.clear();  // 密码不再保留
    green_light_timer_.stop();
    green_light_started_ms_ = -1;
    emit stackProgress(QStringLiteral("stopped"),
                       QStringLiteral("PC 仿真已请求停止（Orin HIL 保持常驻）"));
  }
  const QString stop_script = find_release_script(QStringLiteral("stop_hil.sh"));
  if (hilManagerActive()) {
    hil_stop_requested_ = true;
    emit logLine(QStringLiteral("停止"),
                 QStringLiteral("正在停止 PC 后台栈（Orin HIL/CAN 保持常驻）"));
    hil_manager_.terminate();
    return;
  }
  if (!stop_script.isEmpty()) {
    stop_release_stack(stop_script);
    return;
  }
  emit logLine(QStringLiteral("停止"),
               QStringLiteral("按桥接 → CARLA 顺序停止本 GUI 启动的本机进程"));
  stopBridge();
  if (carla_.active()) {
    emit logLine(QStringLiteral("停止"), QStringLiteral("正在停止本 GUI 启动的 CARLA"));
    carla_.stop();
  } else if (external_carla_) {
    emit logLine(QStringLiteral("停止"),
                 QStringLiteral("外部 CARLA 不属于本 GUI，未发送停止信号"));
    emit carlaChanged(carla_.state(), carla_ready_,
                      QStringLiteral("外部实例 · 未停止"));
  }
}

void ProcessManager::flashMcuFirmware(const QString& firmware_path) {
  if (firmware_path.isEmpty()) {
    const QString detail = QStringLiteral("未指定固件路径，已跳过烧录");
    emit logLine(QStringLiteral("MCU"), detail);
    emit flashFinished(false, detail);
    return;
  }
  if (orin_.busy()) {
    const QString detail = QStringLiteral("Orin 命令正在跑，请等待结束再烧录");
    emit logLine(QStringLiteral("MCU"), detail);
    emit flashFinished(false, detail);
    return;
  }
  emit stackProgress(QStringLiteral("mcu_flash"),
                     QStringLiteral("烧录 F280025C：%1").arg(firmware_path));
  // 烧录是本地操作（不需 ssh）；host/user/password 留空，OrinStackManager 会
  // 自动走 build_flash_argv 分支。
  const int result = orin_.start(OrinStackManager::Op::FlashMcu, QString(),
                                 QString(), QString(), firmware_path, 0);
  if (result != 0) {
    emit flashFinished(false,
                       QStringLiteral("烧录进程启动被拒绝：rc=%1").arg(result));
    return;
  }
  // Phase 2 hardening：烧录不挂超时。若 dslite 中途卡死,GUI 会永远卡在 busy。
  // dslite 是原子操作,不能 SIGTERM 中途打断(可能 brick 芯片),所以
  // 超时后只能整个进程组 SIGKILL,并显式告知用户失败原因。
  flash_timeout_timer_.setSingleShot(true);
  flash_timeout_timer_.setInterval(180000);  // 180s
  flash_timeout_timer_.start();
  emit logLine(QStringLiteral("MCU"),
               QStringLiteral("烧录超时已设置为 180s,超时后将 SIGKILL dslite 进程组"));
}

void ProcessManager::stopBridge() {
  bridge_pending_ = false;
  if (external_bridge_ && bridge_.state() == ProcState::Stopped) {
    emit logLine(QStringLiteral("桥接"),
                 QStringLiteral("外部 carla_bridge 不属于本 GUI，未发送停止信号"));
    emit bridgeChanged(ProcState::Running, QStringLiteral("外部实例 · 未停止"));
    return;
  }
  if (bridge_.active()) {
    emit logLine(QStringLiteral("停止"), QStringLiteral("正在停止本 GUI 启动的桥接"));
  }
  bridge_.stop();
}

void ProcessManager::start_sil_stack(const LaunchConfig& config) {
  if (hilManagerActive()) return;
  const QString script = find_sil_script();
  if (script.isEmpty()) {
    const QString detail =
        QStringLiteral("找不到 scripts/run_sil_fallback.sh；可设置 ADAS_SIL_ROOT=/home/xxs/bowen_ADAS");
    emit logLine(QStringLiteral("SIL"), detail);
    emit carlaChanged(ProcState::Failed, false, detail);
    emit bridgeChanged(ProcState::Failed, detail);
    return;
  }

  pending_config_ = config;
  hil_stop_requested_ = false;
  hil_output_buffer_.clear();
  hil_error_buffer_.clear();
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.insert(QStringLiteral("ADAS_GUI_MODE"), QStringLiteral("sil"));
  hil_manager_.setProcessEnvironment(env);
  const QString scenario = sil_scenario_name(config.scenario);
  const QStringList args{QStringLiteral("--scenario"), scenario};
  emit logLine(QStringLiteral("SIL"),
               QStringLiteral("$ %1 %2 (ROS_DOMAIN_ID=%3)")
                   .arg(script, args.join(' '), env.value(QStringLiteral("ROS_DOMAIN_ID"))));
  emit stackProgress(QStringLiteral("sil"),
                    QStringLiteral("启动 SIL 闭环：%1").arg(scenario));
  hil_manager_.start(script, args);
}

void ProcessManager::start_release_stack(const LaunchConfig& config,
                                         const QString& script) {
  if (hilManagerActive() || stop_helper_.state() != QProcess::NotRunning) return;
  pending_config_ = config;
  hil_stop_requested_ = false;
  hil_output_buffer_.clear();
  hil_error_buffer_.clear();
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.insert(QStringLiteral("CARLA_ROOT"), config.carla_root);
  env.insert(QStringLiteral("TOWN"), config.town);
  env.insert(QStringLiteral("SCENARIO"), config.scenario);
  env.insert(QStringLiteral("CARLA_ARGS"), carla_arguments(config).join(' '));
  hil_manager_.setProcessEnvironment(env);
  QStringList args{QStringLiteral("--skip-gui")};
  args.append(bridge_arguments(config).mid(3));
  emit logLine(QStringLiteral("HIL"),
               QStringLiteral("$ %1 %2").arg(script, args.join(' ')));
  hil_manager_.start(script, args);
}

void ProcessManager::stop_release_stack(const QString& script) {
  if (stop_helper_.state() != QProcess::NotRunning) {
    // Phase 2 hardening：用户连点"停止完整系统"时给出显式反馈,
    // 而非静默 return（之前是死按钮行为）。
    emit logLine(QStringLiteral("停止"),
                 QStringLiteral("停止请求已合并到进行中的停止流程,无需重复触发"));
    emit stackProgress(QStringLiteral("stopping"),
                       QStringLiteral("停止流程进行中,请等待结束"));
    return;
  }
  stop_output_buffer_.clear();
  stop_error_buffer_.clear();
  emit logLine(QStringLiteral("停止"), QStringLiteral("$ %1").arg(script));
  emit carlaChanged(ProcState::Stopping, false, QStringLiteral("停止完整系统中"));
  emit bridgeChanged(ProcState::Stopping, QStringLiteral("停止完整系统中"));
  stop_helper_.start(script);
}

void ProcessManager::forward_hil_output(QProcess& process, QByteArray& buffer,
                                        bool stderr) {
  buffer += stderr ? process.readAllStandardError() : process.readAllStandardOutput();
  int newline;
  while ((newline = buffer.indexOf('\n')) >= 0) {
    const QString line = QString::fromUtf8(buffer.left(newline)).trimmed();
    buffer.remove(0, newline + 1);
    if (!line.isEmpty()) {
      emit logLine(stderr ? QStringLiteral("ERROR") : QStringLiteral("HIL"), line);
    }
  }
}

void ProcessManager::start_readiness_probe() {
  if (readiness_running_) return;  // 已有一次校验在跑，不重复拉起
  const QString script = find_carla_readiness_script();
  if (script.isEmpty()) {
    emit carlaChanged(carla_.state(), false,
                      QStringLiteral("找不到 carla_readiness.py（可设置 ADAS_PC_ROOT 环境变量指向 "
                                     "adas_pc 目录），无法校验 CARLA 是否真正就绪，拒绝启桥"));
    return;
  }
  readiness_output_path_ =
      QDir(QDir::tempPath())
          .filePath(QStringLiteral("adas_gui_carla_readiness_%1.json")
                        .arg(QCoreApplication::applicationPid()));
  QFile::remove(readiness_output_path_);
  emit logLine(QStringLiteral("CARLA"),
              QStringLiteral("校验 CARLA 就绪（RPC/版本/世界/稳定窗口，同 "
                             "start_pc_stack_clean.sh wait_carla_ready）…"));
  readiness_running_ = true;
  readiness_generation_ = startup_generation_;
  readiness_started_ms_ = QDateTime::currentMSecsSinceEpoch();
  readiness_stdout_.clear();
  readiness_stderr_.clear();
  readiness_probe_.setProcessChannelMode(QProcess::SeparateChannels);
  const QStringList args{
      script, QStringLiteral("--host"), QStringLiteral("127.0.0.1"),
      QStringLiteral("--port"), QString::number(pending_config_.carla_port),
      QStringLiteral("--expected-town"), pending_config_.town,
      QStringLiteral("--timeout"), QString::number(kCarlaReadinessTimeoutSeconds),
      QStringLiteral("--output"), readiness_output_path_};
  emit logLine(
      QStringLiteral("STARTUP"),
      QStringLiteral("[STARTUP][CARLA_READINESS_REQUESTED] python=/usr/bin/python3 "
                     "script=%1 args=%2 rpc_host=127.0.0.1 rpc_port=%3 "
                     "generation=%4")
          .arg(script, args.mid(1).join(' '))
          .arg(pending_config_.carla_port)
          .arg(readiness_generation_));
  readiness_timeout_timer_.start((kCarlaReadinessTimeoutSeconds + 5) * 1000);
  readiness_probe_.start(
      QStringLiteral("/usr/bin/python3"), args);
}

void ProcessManager::stop_readiness_probe() {
  if (!readiness_running_) return;
  readiness_timeout_timer_.stop();
  readiness_running_ = false;
  if (readiness_probe_.state() != QProcess::NotRunning) {
    readiness_probe_.kill();
    readiness_probe_.waitForFinished(1000);
  }
}

// ===== 全流程编排 =====

bool ProcessManager::full_stack_running() const {
  return full_stage_ != FullStage::Idle &&
         full_stage_ != FullStage::Complete &&
         full_stage_ != FullStage::Failed;
}

void ProcessManager::fail_full_stack(const QString& stage,
                                     const QString& detail) {
  full_stage_ = FullStage::Failed;
  green_light_timer_.stop();
  green_light_started_ms_ = -1;
  // 清密码：避免堆在成员里被序列化或日志泄露
  full_orin_password_.clear();
  emit stackProgress(QStringLiteral("failed"),
                     QStringLiteral("❌ 在【%1】失败：%2").arg(stage, detail));
}

void ProcessManager::start_full_stack(const LaunchConfig& config) {
  if (full_stack_running()) {
    emit logLine(QStringLiteral("Error"),
                 QStringLiteral("全流程已在跑，请先停止"));
    return;
  }
  full_config_ = config;

  // 1. 取 Orin 密码
  QString password;
  const auto pw = SecureSettings::instance().loadOrinPassword(
      config.orin_host, config.orin_user, &password);
  if (!pw.ok) {
    // 区分"缺密码（首次运行）"与"权限/IO 错误"：前者弹窗让用户填，
    // 后者直接 fail 避免卡在等用户填的伪状态机里。
    const bool missing = pw.detail.contains(QStringLiteral("不存在"));
    if (missing) {
      full_stage_ = FullStage::Idle;  // 等用户填完再次点击"一键启动全流程"
      emit needsOrinCredentials(config.orin_host, config.orin_user);
    } else {
      fail_full_stack(QStringLiteral("Orin auth"), pw.detail);
    }
    return;
  }
  full_orin_password_ = password;
  full_orin_host_ = config.orin_host;
  full_orin_user_ = config.orin_user;

  // 2. 第一步：ip link set can1 up type can bitrate <bitrate>
  full_stage_ = FullStage::SetupCanLink;
  emit stackProgress(QStringLiteral("can_link"),
                     QStringLiteral("等待 Orin CAN 链路 up（can1@%1）…")
                         .arg(config.can_bitrate));
  const int rc = orin_.start(OrinStackManager::Op::SetupCanLink, full_orin_host_,
                             full_orin_user_, full_orin_password_, {},
                             config.can_bitrate);
  if (rc != 0) {
    fail_full_stack(QStringLiteral("can_link"),
                    QStringLiteral("OrinStackManager 拒绝：rc=%1").arg(rc));
  }
}

void ProcessManager::on_orin_command_finished(OrinStackManager::Op op,
                                              int exit_code,
                                              const QString& detail) {
  // 即使 full_stack_running() 已经为 false（被 stopAll 提前打断），也要把
  // OrinStackManager 状态清掉，否则它的 busy_ 仍为 true，下次 start 会
  // 立即返回 -1（被拒）。这里无脑转发：OrinStackManager 自身会处理
  // finished 信号并 set_busy(false)。
  if (op == OrinStackManager::Op::FlashMcu) {
    // 烧录正常完成 / 失败 → 停掉超时 timer,避免 180s 后误触发 SIGKILL。
    flash_timeout_timer_.stop();
    emit flashFinished(exit_code == 0, detail.isEmpty()
        ? (exit_code == 0 ? QStringLiteral("MCU 固件烧录完成")
                          : QStringLiteral("MCU 固件烧录失败：exit=%1").arg(exit_code))
        : detail);
    return;
  }
  if (!full_stack_running()) return;
  if (exit_code != 0) {
    const char* op_name = "?";
    switch (op) {
      case OrinStackManager::Op::SetupCanLink: op_name = "can_link"; break;
      case OrinStackManager::Op::StartHil:     op_name = "orin_hil"; break;
      case OrinStackManager::Op::EnsureHil:    op_name = "orin_hil"; break;
      case OrinStackManager::Op::CheckCanLink: op_name = "can_link"; break;
      case OrinStackManager::Op::FlashMcu:     op_name = "mcu_flash"; break;
    }
    fail_full_stack(QString::fromLatin1(op_name),
                    QStringLiteral("exit=%1 %2").arg(exit_code).arg(detail));
    return;
  }

  switch (full_stage_) {
    case FullStage::SetupCanLink:
      // CAN 链路 up → systemctl start adas-hil（跑 can_hil.launch.py，含 navigation）
      full_stage_ = FullStage::StartOrinHil;
      emit stackProgress(QStringLiteral("orin_hil"),
                         QStringLiteral("等待 Orin adas-hil 启动（can_hil.launch.py）…"));
      orin_.start(OrinStackManager::Op::StartHil, full_orin_host_, full_orin_user_,
                  full_orin_password_, {}, full_config_.can_bitrate);
      break;

    case FullStage::StartOrinHil:
      // Orin 栈起来了（launch 文件异步拉节点；这里不再等具体节点 ready）
      // → 启 CARLA → readiness → bridge → 绿灯
      full_stage_ = FullStage::WaitReadiness;
      emit stackProgress(QStringLiteral("carla"),
                         QStringLiteral("启动 CARLA…"));
      startCarla(full_config_);
      // CARLA running → readiness probe 会被 ManagedProcess::stateChanged 触发；
      // startCarla 走的是原有路径，桥会自动等 readiness OK 才启。
      // 全流程的 WaitReadiness 钩子在 on_readiness_finished 里。
      // 把 full_stage_ 立即推进到 WaitReadiness 表示"正在等 readiness"。
      // CARLA 进程级失败由 WaitReadiness 阶段的 stateChanged hook 收口；
      // readiness 自身失败则由 on_readiness_finished 收口。
      break;

    case FullStage::StartBridge:
      // 桥 Running → 等绿灯
      full_stage_ = FullStage::WaitGreenLight;
      green_light_started_ms_ = QDateTime::currentMSecsSinceEpoch();
      emit stackProgress(QStringLiteral("green_light"),
                         QStringLiteral("等待 Orin 节点绿灯（MCU status fresh + nav status 已收）…"));
      green_light_timer_.start();
      break;

    default:
      break;
  }
}

void ProcessManager::on_carla_state_changed_for_topology(ProcState state,
                                                         bool ready,
                                                         const QString& detail) {
  (void)ready;  // 拓扑钩子只关心 Failed，不看 ready 标志
  if (full_stage_ != FullStage::WaitReadiness) return;
  // CARLA 进程 Failed（不是 readiness failed），主动触发失败
  if (state == ProcState::Failed) {
    fail_full_stack(QStringLiteral("carla"),
                    QStringLiteral("CARLA 进程失败：%1").arg(detail));
  }
}

void ProcessManager::on_bridge_state_changed_for_topology(ProcState state,
                                                           const QString& detail) {
  if (full_stage_ != FullStage::StartBridge) return;
  if (state == ProcState::Failed) {
    fail_full_stack(QStringLiteral("bridge"),
                    QStringLiteral("桥接进程失败：%1").arg(detail));
  }
}

void ProcessManager::check_green_light() {
  if (full_stage_ != FullStage::WaitGreenLight) {
    green_light_timer_.stop();
    return;
  }
  if (!green_light_check_) {
    // 没有注入绿灯判据 → 跳过等待，直接标记完成。
    // 这让没接 RosBridge 的单元测试 / 早期集成也能跑通。
    green_light_timer_.stop();
    full_stage_ = FullStage::Complete;
    full_orin_password_.clear();
    emit stackProgress(QStringLiteral("complete"),
                       QStringLiteral("全流程就绪（未配置绿灯判据，仅启栈完成）✅"));
    return;
  }
  if (green_light_check_()) {
    green_light_timer_.stop();
    full_stage_ = FullStage::Complete;
    full_orin_password_.clear();
    emit stackProgress(QStringLiteral("complete"),
                       QStringLiteral("全流程就绪 ✅"));
    return;
  }
  // 超时：60s 内 MCU status + actuation_feedback 没 fresh，认为卡住
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  if (green_light_started_ms_ > 0 &&
      (now - green_light_started_ms_) > 60000) {
    green_light_timer_.stop();
    fail_full_stack(QStringLiteral("green_light"),
                    QStringLiteral("60s 内 Orin MCU 节点未上电/未发布 /adas/mcu/status"));
  }
}

void ProcessManager::on_readiness_finished(int exit_code, QProcess::ExitStatus exit_status) {
  if (!readiness_running_) return;  // 被 stop_readiness_probe() 主动打断，结果作废
  readiness_running_ = false;
  readiness_timeout_timer_.stop();
  const qint64 elapsed_ms =
      QDateTime::currentMSecsSinceEpoch() - readiness_started_ms_;
  emit logLine(
      QStringLiteral("STARTUP"),
      QStringLiteral("[STARTUP][CARLA_READINESS_FINISHED] exit_code=%1 "
                     "exit_status=%2 elapsed_ms=%3 generation=%4 "
                     "current_generation=%5")
          .arg(exit_code)
          .arg(static_cast<int>(exit_status))
          .arg(elapsed_ms)
          .arg(readiness_generation_)
          .arg(startup_generation_));
  if (readiness_generation_ != startup_generation_) {
    emit logLine(QStringLiteral("STARTUP"),
                 QStringLiteral("[STARTUP][CARLA_READINESS_STALE_IGNORED]"));
    return;
  }

  QString detail;
  QFile out(readiness_output_path_);
  if (out.open(QIODevice::ReadOnly)) {
    const QJsonObject obj = QJsonDocument::fromJson(out.readAll()).object();
    if (obj.contains(QStringLiteral("reason"))) {
      detail = obj.value(QStringLiteral("reason")).toString();
    }
  }

  emit logLine(QStringLiteral("桥接"),
               QStringLiteral("on_readiness_finished: exit=%1 bridge_pending=%2 full_stage=%3")
                   .arg(exit_code).arg(bridge_pending_).arg(static_cast<int>(full_stage_)));
  if (exit_status == QProcess::NormalExit && exit_code == 0) {
    emit logLine(QStringLiteral("STARTUP"),
                 QStringLiteral("[STARTUP][CARLA_READY_ACCEPTED]"));
    carla_ready_ = true;
    emit carlaChanged(carla_.state(), true,
                      detail.isEmpty()
                          ? QStringLiteral("CARLA 就绪（RPC/世界/稳定窗口校验通过）")
                          : detail);
    if (bridge_pending_) {
      bridge_pending_ = false;
      emit logLine(QStringLiteral("STARTUP"),
                   QStringLiteral("[STARTUP][BRIDGE_START_QUEUED] ros_runtime=humble-container"));
      startBridge(pending_config_);
    }
    // 全流程：CARLA 就绪 → 启桥
    if (full_stage_ == FullStage::WaitReadiness) {
      full_stage_ = FullStage::StartBridge;
      emit stackProgress(QStringLiteral("bridge"),
                         QStringLiteral("CARLA 就绪，启动桥接…"));
      startBridge(full_config_);
    }
  } else {
    carla_ready_ = false;
    emit carlaChanged(carla_.state(), false,
                      QStringLiteral("CARLA 就绪校验未通过：%1")
                          .arg(detail.isEmpty() ? QStringLiteral("见 carla_readiness 输出")
                                                : detail));
    // 全流程：readiness 失败 → 整条停
    if (full_stage_ == FullStage::WaitReadiness) {
      fail_full_stack(QStringLiteral("readiness"),
                      QStringLiteral("CARLA 就绪校验失败 exit=%1").arg(exit_code));
    }
  }
}

}  // namespace adas::gui
